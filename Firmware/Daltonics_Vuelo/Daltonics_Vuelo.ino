// =========================================================================
// Daltonics_Vuelo.ino — Firmware de vuelo con MAQUINA DE ESTADOS
// (ESP32-C3 SuperMini, unidad AIRE — Escuderia Daltonics)
//
// Evolucion de Daltonics_Aire.ino. Agrega:
//   1) Calibracion automatica del giroscopio al encender (bias) — el dron
//      DEBE estar quieto y nivelado durante los primeros ~3 segundos.
//   2) Maquina de estados comandada desde tierra por el campo A del
//      protocolo (antes solo 0/1, ahora 0-3):
//        A=0  DESARMADO   -> motores apagados
//        A=1  IDLE        -> motores girando suave (IDLE_US) SIN despegar
//        A=2  VUELO       -> despegue con rampa suave hacia el throttle
//                            de hover + PID de actitud (se queda nivelado)
//        A=3  ATERRIZAJE  -> rampa descendente automatica hasta idle
//   3) Rampa (slew-rate) de throttle: nunca hay saltos bruscos de gas.
//
// PROTOCOLO: mismo PaqueteControl de siempre. ESP32_PRUEBATIERRA.ino NO
// necesita cambios (su parser del campo A ya acepta 0-3). En tierra usar
// CONTROL/Control_Vuelo_Daltonics.m (teclas E/T/L/Q).
//
// *** SIN ALTIMETRO: el hover mantiene NIVEL (roll/pitch) pero la ALTURA
// *** es lazo abierto. El gas de hover se ajusta fino con W/S desde
// *** MATLAB. Primera prueba: sosten el dron con la mano (lejos de las
// *** helices) o atado, a baja altura, y con la LiPo real — la fuente de
// *** banco puede resetear el ESP32 con los motores armados.
//
// Checklist antes de volar (una sola vez):
//   1) Calibrar ESCs (Firmware/Calibrar_ESCs).
//   2) SIN HELICES: armar (E), inclinar a mano y confirmar por Serial que
//      el motor del lado que baja es el que SUBE de PWM.
//   3) Verificar que IDLE_US hace girar los 4 motores parejos sin empuje.
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= PROTOCOLO =================
struct __attribute__((packed)) PaqueteControl {
    uint16_t throttle;   // 1000-2000 us (en VUELO: gas objetivo de hover)
    uint16_t roll;       // 1000-2000 us, centro 1500
    uint16_t pitch;      // 1000-2000 us, centro 1500
    uint16_t yaw;        // 1000-2000 us, centro 1500 (yaw-RATE)
    uint8_t  armable;    // MODO: 0=desarmado 1=idle 2=vuelo 3=aterrizaje
};

enum Modo : uint8_t { M_DESARMADO = 0, M_IDLE = 1, M_VUELO = 2, M_ATERRIZAJE = 3 };

PaqueteControl comandosDron = {1000, 1500, 1500, 1500, M_DESARMADO};
unsigned long ultimaVezRecibido = 0;
const unsigned long TIMEOUT_FAILSAFE = 500; // ms sin señal -> failsafe

// ================= TELEMETRIA HACIA TIERRA (dron -> operador) =================
// El dron aprende el MAC de Tierra del primer paquete recibido y le manda
// su estado a 10 Hz. Tierra lo reenvia por USB a MATLAB como "TL,...".
struct __attribute__((packed)) PaqueteTelemetria {
    float   rollDeg;   // actitud estimada
    float   pitchDeg;
    float   thrPct;    // gas aplicado ahora (thrActual)
    uint8_t modo;      // 0-3
    uint8_t flags;     // b0=emergencia b1=failsafe b2=aterrizado b3=mpuOK
};
uint8_t macTierra[6];
volatile bool macTierraCapturada = false;
bool peerTierraListo = false;
unsigned long ultimoTelem = 0;

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(PaqueteControl)) {
        memcpy(&comandosDron, incomingData, sizeof(comandosDron));
        ultimaVezRecibido = millis();
        if (!macTierraCapturada) {
            memcpy(macTierra, recv_info->src_addr, 6);
            macTierraCapturada = true;
        }
    }
}

// ================= MPU6050 (pines C3 SuperMini) =================
#define SDA_PIN 20
#define SCL_PIN 21
Adafruit_MPU6050 mpu;
bool mpuOK = false;

// Bias del giroscopio, medido en la calibracion de arranque
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
bool imuCalibrada = false;

// ================= NEOPIXEL =================
#define PIN_NEOPIXEL 10
#define NUM_LEDS 12
Adafruit_NeoPixel aro(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ================= MOTORES (pines/desfases confirmados) =================
Servo motor1, motor2, motor3, motor4;
const int pinM1 = 1, pinM2 = 2, pinM3 = 3, pinM4 = 4;
// M1=delantero-izq  M2=delantero-der  M3=trasero-izq  M4=trasero-der

int desfaseM1 = -25;
int desfaseM2 = 0;
int desfaseM3 = 0;
int desfaseM4 = 0;

const int PWM_MIN_US  = 1000;
const int PWM_MAX_US  = 2000;
const int IDLE_US     = 1120;  // motores girando suave, SIN empuje de despegue
                               // (ajustar: debe girar parejo en los 4 sin que
                               //  el dron "aligere" sobre el piso)

// ================= RAMPAS DE THROTTLE (en % de gas, 0-100) =================
const float RAMPA_SUBIDA_PCT_S  = 8.0f;   // despegue: sube 8%/s (susave)
const float RAMPA_BAJADA_PCT_S  = 12.0f;  // correccion hacia abajo en vuelo
const float RAMPA_ATERR_PCT_S   = 3.5f;   // aterrizaje: baja 3.5%/s
const float PCT_CORTE_ATERR     = 25.0f;  // debajo de esto ya no sustenta ->
                                          // corta a idle (aterrizado)
float thrActual = 0.0f;                    // % de gas aplicado ahora mismo
bool  aterrizado = false;                  // subestado de M_ATERRIZAJE

// ================= GEOMETRIA / EMPUJE (igual que Daltonics_Aire) =================
float DX   = 0.1061f;
float DY   = 0.1061f;
float TMAX = 2.73f;
float CYAW = 0.013f;

// ================= GANANCIAS PID =================
// Punto de partida CONSERVADOR (~mitad del gemelo digital) para el primer
// contacto real: mejor blando que sobre-corregir y voltear.
//
// GANANCIAS DEL GEMELO DIGITAL (DALTONICS_parametros.m, tuneadas en
// simulacion para el dron de 530 g) — meta a la que subir gradualmente
// si la prueba de banco sin helices responde sana:
//   Kp_ang = 0.077   Ki_ang = 0.019   Kd_ang = 0.023   N = 40
//   Kp_yaw = 0.052   Ki_yaw = 0.022
// Orden de tuneo: Kp primero, Kd si oscila, Ki al final y de a poco
// (si se activa Ki, agregar limite al integrador / anti-windup).
float Kp_ang = 0.035f, Ki_ang = 0.0f, Kd_ang = 0.010f, N_filt = 20.0f;
float Kp_yaw = 0.020f, Ki_yaw = 0.0f;

// ================= PID 2-DOF (roll/pitch) y PI (yaw-rate) =================
struct PID2DOF {
    float Kp, Ki, Kd, N;
    float integ = 0, dFilt = 0, prevMeas = 0;
    bool primed = false;
    float update(float ref, float meas, float dt) {
        if (!primed) { prevMeas = meas; primed = true; }
        float error = ref - meas;
        integ += Ki * error * dt;
        float rawDeriv = (meas - prevMeas) / dt;
        dFilt += (rawDeriv - dFilt) * constrain(N * dt, 0.0f, 1.0f);
        prevMeas = meas;
        return Kp * error + integ - Kd * dFilt;
    }
    void reset() { integ = 0; dFilt = 0; primed = false; }
};

struct PIControlador {
    float Kp, Ki;
    float integ = 0;
    float update(float ref, float meas, float dt) {
        float error = ref - meas;
        integ += Ki * error * dt;
        return Kp * error + integ;
    }
    void reset() { integ = 0; }
};

PID2DOF pidRoll, pidPitch;
PIControlador pidYaw;

// ================= ESTIMACION DE ACTITUD =================
float rollEst = 0, pitchEst = 0; // rad
const float ALPHA = 0.98f;

// ================= MEZCLADORA =================
float Mix[4][4];

void invertir4x4(float A[4][4], float Inv[4][4]) {
    float aug[4][8];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) aug[i][j] = A[i][j];
        for (int j = 0; j < 4; j++) aug[i][4 + j] = (i == j) ? 1.0f : 0.0f;
    }
    for (int col = 0; col < 4; col++) {
        int piv = col;
        for (int r = col + 1; r < 4; r++)
            if (fabs(aug[r][col]) > fabs(aug[piv][col])) piv = r;
        if (piv != col) for (int j = 0; j < 8; j++) { float t = aug[col][j]; aug[col][j] = aug[piv][j]; aug[piv][j] = t; }
        float d = aug[col][col];
        for (int j = 0; j < 8; j++) aug[col][j] /= d;
        for (int r = 0; r < 4; r++) {
            if (r == col) continue;
            float f = aug[r][col];
            for (int j = 0; j < 8; j++) aug[r][j] -= f * aug[col][j];
        }
    }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) Inv[i][j] = aug[i][4 + j];
}

void construirMezcladora() {
    float A[4][4] = {
        { 1,     1,     1,     1   },
        { DY,   -DY,    DY,   -DY  },
        {-DX,   -DX,    DX,    DX  },
        {-CYAW,  CYAW,  CYAW, -CYAW}
    };
    invertir4x4(A, Mix);
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ================= SETPOINTS =================
void leerSetpoints(float &thrPct, float &rollRef, float &pitchRef, float &yawRateRef) {
    thrPct     = constrain(map(comandosDron.throttle, 1000, 2000, 0, 100), 0, 100);
    rollRef    = (constrain((int)comandosDron.roll,  1000, 2000) - 1500) / 500.0f * 25.0f  * DEG_TO_RAD;
    pitchRef   = (constrain((int)comandosDron.pitch, 1000, 2000) - 1500) / 500.0f * 25.0f  * DEG_TO_RAD;
    yawRateRef = (constrain((int)comandosDron.yaw,   1000, 2000) - 1500) / 500.0f * 120.0f * DEG_TO_RAD;
}

// ================= CORTE POR INCLINACION EXTREMA =================
const float DANGER_ANGLE_RAD = 45.0f * DEG_TO_RAD;
const int   DANGER_DEBOUNCE = 25; // ~100ms a 250Hz
int contadorPeligro = 0;
bool emergencia = false; // requiere desarmar (A=0) desde tierra para limpiar

// ---------------------------------------------------------------------
void aplicarPWM(int m1, int m2, int m3, int m4) {
    motor1.writeMicroseconds(constrain(m1, PWM_MIN_US, PWM_MAX_US) + desfaseM1);
    motor2.writeMicroseconds(constrain(m2, PWM_MIN_US, PWM_MAX_US) + desfaseM2);
    motor3.writeMicroseconds(constrain(m3, PWM_MIN_US, PWM_MAX_US) + desfaseM3);
    motor4.writeMicroseconds(constrain(m4, PWM_MIN_US, PWM_MAX_US) + desfaseM4);
}

void apagarMotores() {
    motor1.writeMicroseconds(PWM_MIN_US);
    motor2.writeMicroseconds(PWM_MIN_US);
    motor3.writeMicroseconds(PWM_MIN_US);
    motor4.writeMicroseconds(PWM_MIN_US);
}

void motoresIdle() {
    aplicarPWM(IDLE_US, IDLE_US, IDLE_US, IDLE_US);
}

// ================= CALIBRACION IMU AL ARRANQUE =================
// El dron debe estar QUIETO y NIVELADO. Promedia el giroscopio para sacar
// el bias, e inicializa rollEst/pitchEst con el acelerometro para que el
// filtro complementario no arranque desde cero si el piso no es perfecto.
void calibrarIMU() {
    const int N = 500;
    float sx = 0, sy = 0, sz = 0;
    sensors_event_t acc, gyr, temp;

    Serial.println("Calibrando IMU... NO MUEVAS EL DRON (3 s)");
    for (int i = 0; i < N; i++) {
        mpu.getEvent(&acc, &gyr, &temp);
        sx += gyr.gyro.x; sy += gyr.gyro.y; sz += gyr.gyro.z;
        // LED: barrido violeta mientras calibra
        if (i % 40 == 0) {
            aro.clear();
            aro.setPixelColor((i / 40) % NUM_LEDS, aro.Color(140, 0, 255));
            aro.show();
        }
        delay(5);
    }
    gyroBiasX = sx / N; gyroBiasY = sy / N; gyroBiasZ = sz / N;

    mpu.getEvent(&acc, &gyr, &temp);
    rollEst  = atan2(acc.acceleration.y, acc.acceleration.z);
    pitchEst = atan2(-acc.acceleration.x,
                      sqrt(acc.acceleration.y * acc.acceleration.y +
                           acc.acceleration.z * acc.acceleration.z));
    imuCalibrada = true;
    aro.clear(); aro.show();
    Serial.printf("IMU calibrada. Bias gyro [rad/s]: %.4f %.4f %.4f | roll0:%.1f pitch0:%.1f deg\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ, rollEst * RAD_TO_DEG, pitchEst * RAD_TO_DEG);
}

// ================= INICIALIZACION DE ESCs =================
// Manda minimo sostenido para que los ESC armen (beeps) antes de aceptar
// cualquier modo. Los motores quedan LISTOS pero parados.
void inicializarESCs() {
    Serial.println("Inicializando ESCs (pulso minimo 3 s)...");
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
        apagarMotores();
        // LED: barrido ambar
        aro.clear();
        aro.setPixelColor(((millis() - t0) / 250) % NUM_LEDS, aro.Color(255, 150, 0));
        aro.show();
        delay(20);
    }
    aro.clear(); aro.show();
    Serial.println("ESCs listos.");
}

// ---------------------------------------------------------------------
unsigned long ultimoDebug = 0;
unsigned long tPrevControl = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    // --- ESP-NOW primero ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error critico: no se pudo inicializar ESP-NOW");
    }
    esp_now_register_recv_cb(onDataRecv);

    // --- NeoPixel ---
    aro.begin();
    aro.setBrightness(60);

    // --- ESCs: attach y secuencia de inicializacion ---
    motor1.attach(pinM1, PWM_MIN_US, PWM_MAX_US);
    motor2.attach(pinM2, PWM_MIN_US, PWM_MAX_US);
    motor3.attach(pinM3, PWM_MIN_US, PWM_MAX_US);
    motor4.attach(pinM4, PWM_MIN_US, PWM_MAX_US);
    inicializarESCs();

    // --- IMU + calibracion ---
    Wire.begin(SDA_PIN, SCL_PIN);
    mpuOK = mpu.begin();
    if (!mpuOK) {
        Serial.println("Error: MPU6050 no detectado. NO se podra armar.");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
        calibrarIMU();
    }

    // --- PID / mezcladora ---
    pidRoll.Kp = Kp_ang;  pidRoll.Ki = Ki_ang;  pidRoll.Kd = Kd_ang;  pidRoll.N = N_filt;
    pidPitch.Kp = Kp_ang; pidPitch.Ki = Ki_ang; pidPitch.Kd = Kd_ang; pidPitch.N = N_filt;
    pidYaw.Kp = Kp_yaw;   pidYaw.Ki = Ki_yaw;
    construirMezcladora();

    Serial.println("Daltonics_Vuelo listo. Esperando modo desde Tierra:");
    Serial.println("  A=1 idle | A=2 despegue/hover | A=3 aterrizaje | A=0 desarmado");
    tPrevControl = micros();
}

// ---------------------------------------------------------------------
void loop() {
    unsigned long ahora = micros();
    float dt = (ahora - tPrevControl) / 1e6f;
    tPrevControl = ahora;
    if (dt <= 0 || dt > 0.5f) dt = 0.004f;

    // --- Failsafe: sin señal -> estado seguro (motores fuera) ---
    bool senalPerdida = (millis() - ultimaVezRecibido) > TIMEOUT_FAILSAFE;
    if (senalPerdida) {
        comandosDron.throttle = 1000;
        comandosDron.roll = 1500; comandosDron.pitch = 1500; comandosDron.yaw = 1500;
        comandosDron.armable = M_DESARMADO;
    }

    Modo modo = (Modo)comandosDron.armable;
    if (modo > M_ATERRIZAJE) modo = M_DESARMADO; // valor corrupto -> seguro

    // --- Reset de emergencia: tierra debe pasar por A=0 ---
    if (modo == M_DESARMADO) { emergencia = false; }

    // --- Leer IMU y estimar actitud (con bias restado) ---
    float yawRateMeas = 0;
    if (mpuOK) {
        sensors_event_t acc, gyr, temp;
        mpu.getEvent(&acc, &gyr, &temp);

        float gx = gyr.gyro.x - gyroBiasX;
        float gy = gyr.gyro.y - gyroBiasY;
        float gz = gyr.gyro.z - gyroBiasZ;

        float accelRoll  = atan2(acc.acceleration.y, acc.acceleration.z);
        float accelPitch = atan2(-acc.acceleration.x,
                                  sqrt(acc.acceleration.y * acc.acceleration.y +
                                       acc.acceleration.z * acc.acceleration.z));
        rollEst  = ALPHA * (rollEst  + gx * dt) + (1 - ALPHA) * accelRoll;
        pitchEst = ALPHA * (pitchEst + gy * dt) + (1 - ALPHA) * accelPitch;
        yawRateMeas = gz;

        if (fabs(rollEst) > DANGER_ANGLE_RAD || fabs(pitchEst) > DANGER_ANGLE_RAD) {
            contadorPeligro++;
            if (contadorPeligro >= DANGER_DEBOUNCE && (modo == M_VUELO || modo == M_ATERRIZAJE)) {
                emergencia = true;
                Serial.println("¡PELIGRO! Inclinacion extrema -> motores apagados.");
            }
        } else {
            contadorPeligro = 0;
        }
    }

    // --- Setpoints ---
    float thrCmdPct, rollRef, pitchRef, yawRateRef;
    leerSetpoints(thrCmdPct, rollRef, pitchRef, yawRateRef);

    bool puedeVolar = mpuOK && imuCalibrada && !senalPerdida && !emergencia;

    // =================== MAQUINA DE ESTADOS ===================
    if (!puedeVolar || modo == M_DESARMADO) {
        // ---------- DESARMADO / EMERGENCIA / FAILSAFE ----------
        apagarMotores();
        thrActual = 0;
        aterrizado = false;
        pidRoll.reset(); pidPitch.reset(); pidYaw.reset();

        if (emergencia)        aro.fill(aro.Color(255, 0, 0));   // rojo
        else if (senalPerdida) aro.fill(aro.Color(255, 60, 0));  // naranja
        else if (!mpuOK || !imuCalibrada) aro.fill(aro.Color(255, 0, 255)); // magenta
        else                   aro.fill(aro.Color(0, 40, 0));    // verde tenue

    } else if (modo == M_IDLE) {
        // ---------- IDLE: motores girando, sin despegar ----------
        motoresIdle();
        thrActual = (IDLE_US - PWM_MIN_US) / 10.0f; // % equivalente al idle
        aterrizado = false;
        pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
        aro.fill(aro.Color(0, 255, 60)); // verde brillante = armado en idle

    } else {
        // ---------- VUELO (A=2) o ATERRIZAJE (A=3) ----------
        float thrObjetivo;
        if (modo == M_VUELO) {
            aterrizado = false;
            thrObjetivo = thrCmdPct; // gas objetivo desde tierra (hover +/- trim W/S)
        } else { // M_ATERRIZAJE
            if (thrActual <= PCT_CORTE_ATERR) aterrizado = true;
            thrObjetivo = aterrizado ? (IDLE_US - PWM_MIN_US) / 10.0f : 0.0f;
        }

        // Rampa de throttle (slew-rate)
        float maxSubida, maxBajada;
        if (modo == M_ATERRIZAJE && !aterrizado) {
            maxSubida = 0;                            // aterrizando no se sube
            maxBajada = RAMPA_ATERR_PCT_S * dt;
        } else {
            maxSubida = RAMPA_SUBIDA_PCT_S * dt;
            maxBajada = RAMPA_BAJADA_PCT_S * dt;
        }
        float delta = thrObjetivo - thrActual;
        if (delta >  maxSubida) delta =  maxSubida;
        if (delta < -maxBajada) delta = -maxBajada;
        thrActual += delta;

        if (modo == M_ATERRIZAJE && aterrizado) {
            // Ya toco piso (o gas bajo el minimo de sustentacion): idle
            motoresIdle();
            pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
            aro.fill(aro.Color(0, 120, 255)); // celeste = aterrizado, en idle
        } else {
            // PID de actitud activo (en aterrizaje se mantiene nivelado
            // mientras baja: refs de roll/pitch siguen viniendo de tierra,
            // que en ese modo manda 0)
            float tauX = pidRoll.update(rollRef, rollEst, dt);
            float tauY = pidPitch.update(pitchRef, pitchEst, dt);
            float tauZ = pidYaw.update(yawRateRef, yawRateMeas, dt);

            float Ftotal = thrActual * (4.0f * TMAX / 100.0f);
            float u[4] = {Ftotal, tauX, tauY, tauZ};
            float T[4];
            for (int i = 0; i < 4; i++) {
                T[i] = Mix[i][0]*u[0] + Mix[i][1]*u[1] + Mix[i][2]*u[2] + Mix[i][3]*u[3];
                T[i] = constrain(T[i], 0.0f, TMAX);
            }
            int pulso[4];
            for (int i = 0; i < 4; i++)
                pulso[i] = (int)constrain(mapFloat(T[i], 0.0f, TMAX, PWM_MIN_US, PWM_MAX_US), PWM_MIN_US, PWM_MAX_US);
            aplicarPWM(pulso[0], pulso[1], pulso[2], pulso[3]);

            if (modo == M_VUELO) aro.fill(aro.Color(0, 150, 255));  // azul = volando
            else                 aro.fill(aro.Color(255, 255, 0));  // amarillo = aterrizando
        }
    }
    aro.show();

    // --- Telemetria hacia Tierra ~10 Hz ---
    if (macTierraCapturada && !peerTierraListo) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, macTierra, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) == ESP_OK) peerTierraListo = true;
    }
    if (peerTierraListo && millis() - ultimoTelem > 100) {
        ultimoTelem = millis();
        PaqueteTelemetria tl;
        tl.rollDeg  = rollEst * RAD_TO_DEG;
        tl.pitchDeg = pitchEst * RAD_TO_DEG;
        tl.thrPct   = thrActual;
        tl.modo     = (uint8_t)modo;
        tl.flags    = (emergencia ? 1 : 0) | (senalPerdida ? 2 : 0) |
                      (aterrizado ? 4 : 0) | (mpuOK ? 8 : 0);
        esp_now_send(macTierra, (uint8_t *)&tl, sizeof(tl));
    }

    // --- Debug ~10 Hz ---
    if (millis() - ultimoDebug > 100) {
        ultimoDebug = millis();
        Serial.printf("MODO:%d FS:%d EMG:%d ATZ:%d | thr:%.1f%% obj:%.1f%% | roll:%.1f pitch:%.1f deg\n",
            modo, senalPerdida, emergencia, aterrizado,
            thrActual, thrCmdPct, rollEst * RAD_TO_DEG, pitchEst * RAD_TO_DEG);
    }

    delay(4); // ~200-250 Hz
}
