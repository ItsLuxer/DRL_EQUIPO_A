// =========================================================================
// Daltonics_Aire.ino — Firmware de vuelo CON PID (ESP32, unidad AIRE)
//
// Fase 2: reemplaza la mezcladora manual (stick -> motores sin corregir
// nada) por un lazo cerrado real: MPU6050 -> estimacion de actitud -> PID
// (misma estructura que el gemelo digital, ver DALTONICS_parametros.m /
// DALTONICS_dinamica.m en Dron_Digital... perdon, en Dron_Daltonics/
// MATLAB_DRIVE) -> mezcladora fisica -> motores.
//
// *** ANTES DE PROBAR ESTO CON HELICES ***
// 1) Calibrar los 4 ESCs (ver Firmware/Calibrar_ESCs/Calibrar_ESCs.ino).
// 2) Probar con la BATERIA 3S real, no con la fuente de banco (una fuente
//    que se cae de amperaje a media carga puede resetear el ESP32 con los
//    motores armados).
// 3) Repetir la prueba de banco SIN HELICES: armar, inclinar el conjunto a
//    mano, y confirmar por Serial que el motor del lado que se hunde es el
//    que sube de PWM (si es al reves, el dron va a acelerar la caida en
//    vez de corregirla -> NO poner helices hasta que esto se vea bien).
// 4) Recien despues, primera prueba con helices sostenido con la mano
//    (lejos de las helices) a throttle bajo, antes de soltar en el piso.
//
// Pines, desfases de motor y NeoPixel: igual que el sketch original del
// usuario. Protocolo ESP-NOW: igual que ESP32_PRUEBATIERRA.ino /
// CONTROL/Enviar_Mando_ESP32_Tierra.m (sin cambios).
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= PROTOCOLO (igual que ESP32_PRUEBATIERRA.ino / Enviar_Mando_ESP32_Tierra.m) =================
struct __attribute__((packed)) PaqueteControl {
    uint16_t throttle;   // 1000-2000 us
    uint16_t roll;       // 1000-2000 us, centro 1500
    uint16_t pitch;      // 1000-2000 us, centro 1500
    uint16_t yaw;        // 1000-2000 us, centro 1500 (yaw-RATE, no angulo)
    uint8_t  armable;    // 0 = desarmado, 1 = armado
};

PaqueteControl comandosDron = {1000, 1500, 1500, 1500, 0};
unsigned long ultimaVezRecibido = 0;
const unsigned long TIMEOUT_FAILSAFE = 500; // ms sin señal -> failsafe

// NOTA: firma para nucleos ESP32 basados en ESP-IDF 5 (Arduino core 3.x).
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(PaqueteControl)) {
        memcpy(&comandosDron, incomingData, sizeof(comandosDron));
        ultimaVezRecibido = millis();
    }
}

// ================= CONFIGURACION MPU6050 (pines del usuario) =================
#define SDA_PIN 20
#define SCL_PIN 21
Adafruit_MPU6050 mpu;
bool mpuOK = false;

// ================= CONFIGURACION NEOPIXEL =================
#define PIN_NEOPIXEL 10
#define NUM_LEDS 12
Adafruit_NeoPixel aro(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ================= CONFIGURACION MOTORES (pines/desfases del usuario) =================
Servo motor1, motor2, motor3, motor4;
const int pinM1 = 1, pinM2 = 2, pinM3 = 3, pinM4 = 4;
// Orden fisico confirmado: M1=delantero-izq M2=delantero-der M3=trasero-izq M4=trasero-der

int desfaseM1 = -25;
int desfaseM2 = 0;
int desfaseM3 = 0;
int desfaseM4 = 0;

const int PWM_MIN_US = 1000;
const int PWM_MAX_US = 2000;

// ================= GEOMETRIA / EMPUJE (igual que DALTONICS_parametros.m) =================
// TODO: siguen siendo estimaciones (ver DALTONICS_parametros.m). No se
// "recalibraron" con el dato de banco de 1450us porque esa prueba se hizo
// con la fuente que se cae de amperaje -> ese dato no es confiable todavia.
float DX   = 0.1061f;  // m
float DY   = 0.1061f;  // m
float TMAX = 2.73f;    // N por motor [EST]
float CYAW = 0.013f;   // coef. par de arrastre [EST]

// ================= GANANCIAS PID =================
// *** PUNTO DE PARTIDA DELIBERADAMENTE BAJO PARA LA PRIMERA PRUEBA REAL ***
// Son MAS bajas que las del gemelo digital (DALTONICS_parametros.m) a
// proposito: mejor que corrija poco y se sienta "blando" a que sobre-
// corrija y voltee. Sin integral todavia (Ki=0) para el primer intento.
//
// COMO SUBIRLAS (en este orden, en banco SIN HELICES primero):
//   1) Sube Kp_ang de a poco hasta que al inclinar el dron a mano sientas
//      que "empuja de regreso" con fuerza pero SIN vibrar/oscilar solo.
//   2) Si oscila (tiembla) al soltarlo, sube Kd_ang un poco para amortiguar.
//   3) Solo al final, si se queda "recostado" en vez de nivelarse del
//      todo, sube Ki_ang un poco (valores chicos, esto es lo mas
//      facil de hacer inestable).
float Kp_ang = 0.035f, Ki_ang = 0.0f, Kd_ang = 0.010f, N_filt = 20.0f;
float Kp_yaw = 0.020f, Ki_yaw = 0.0f;

// ================= PID 2-DOF (roll/pitch) y PI (yaw-rate) =================
// Misma estructura que el gemelo digital: derivada sobre la medicion (no
// sobre el error) para evitar "derivative kick" al mover el stick.
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

// ================= ESTIMACION DE ACTITUD (filtro complementario) =================
// TODO: validar signo/orientacion contra el montaje real del MPU6050 en
// el chasis (paso 3 de la lista de arriba, con el dron en la mano SIN
// HELICES, mirando los angulos por Serial mientras inclinas a mano).
float rollEst = 0, pitchEst = 0; // rad
const float ALPHA = 0.98f;

// ================= MEZCLADORA (igual que DALTONICS_parametros.m / DALTONICS_dinamica.m) =================
// Orden fisico real: M1=FL(+x,+y) M2=FR(+x,-y) M3=RL(-x,+y) M4=RR(-x,-y).
// Pares diagonales para yaw: M1/M4 vs M2/M3 (por el orden de arriba).
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

// ================= LECTURA DE SETPOINTS DESDE EL MANDO =================
void leerSetpoints(float &thrPct, float &rollRef, float &pitchRef, float &yawRateRef, bool &armado) {
    thrPct     = constrain(map(comandosDron.throttle, 1000, 2000, 0, 100), 0, 100);
    rollRef    = (constrain((int)comandosDron.roll,  1000, 2000) - 1500) / 500.0f * 25.0f  * DEG_TO_RAD;
    pitchRef   = (constrain((int)comandosDron.pitch, 1000, 2000) - 1500) / 500.0f * 25.0f  * DEG_TO_RAD;
    yawRateRef = (constrain((int)comandosDron.yaw,   1000, 2000) - 1500) / 500.0f * 120.0f * DEG_TO_RAD;
    armado     = (comandosDron.armable == 1);
}

// ================= CORTE DE SEGURIDAD POR INCLINACION =================
// Ahora usa el angulo ESTIMADO (rollEst/pitchEst), no el acelerometro
// crudo -- mas confiable, ya que el acelerometro crudo tambien reacciona a
// aceleraciones lineales (no solo inclinacion) y puede dar falsos
// positivos/negativos durante maniobras.
const float DANGER_ANGLE_RAD = 45.0f * DEG_TO_RAD; // margen sobre el maximo comandado (+-25 deg)
const int   DANGER_DEBOUNCE = 25; // ~100ms a 250Hz
int contadorPeligro = 0;
bool emergencia = false; // se queda activo hasta que tierra desarme y vuelva a armar

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

void animacionArmado(int tiempoMs) {
    int iteraciones = tiempoMs / 100;
    for (int i = 0; i < iteraciones; i++) {
        aro.clear();
        aro.setPixelColor(i % NUM_LEDS, aro.Color(255, 150, 0));
        aro.show();
        delay(100);
    }
    aro.clear();
    aro.show();
}

// ---------------------------------------------------------------------
unsigned long ultimoDebug = 0;
unsigned long tPrevControl = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    // --- ESP-NOW primero, para que empiece a recibir cuanto antes ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error critico: no se pudo inicializar ESP-NOW");
    }
    esp_now_register_recv_cb(onDataRecv);

    // --- IMU ---
    Wire.begin(SDA_PIN, SCL_PIN);
    mpuOK = mpu.begin();
    if (!mpuOK) {
        Serial.println("Error: MPU6050 no detectado. Revisa conexiones.");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); // filtro contra vibraciones
    }

    // --- NeoPixel ---
    aro.begin();
    aro.setBrightness(60);

    // --- ESCs ---
    motor1.attach(pinM1, PWM_MIN_US, PWM_MAX_US);
    motor2.attach(pinM2, PWM_MIN_US, PWM_MAX_US);
    motor3.attach(pinM3, PWM_MIN_US, PWM_MAX_US);
    motor4.attach(pinM4, PWM_MIN_US, PWM_MAX_US);
    apagarMotores();

    // --- PID / mezcladora ---
    pidRoll.Kp = Kp_ang;  pidRoll.Ki = Ki_ang;  pidRoll.Kd = Kd_ang;  pidRoll.N = N_filt;
    pidPitch.Kp = Kp_ang; pidPitch.Ki = Ki_ang; pidPitch.Kd = Kd_ang; pidPitch.N = N_filt;
    pidYaw.Kp = Kp_yaw;   pidYaw.Ki = Ki_yaw;
    construirMezcladora();

    Serial.println("Iniciando secuencia de armado...");
    animacionArmado(5000);
    Serial.println("Listo. Esperando mando desde Tierra (ESP-NOW)...");

    tPrevControl = micros();
}

// ---------------------------------------------------------------------
void loop() {
    unsigned long ahora = micros();
    float dt = (ahora - tPrevControl) / 1e6f;
    tPrevControl = ahora;
    if (dt <= 0 || dt > 0.5f) dt = 0.004f; // primer loop / salto raro de reloj

    // --- Failsafe: sin señal por mas de 500ms -> estado seguro ---
    bool senalPerdida = (millis() - ultimaVezRecibido) > TIMEOUT_FAILSAFE;
    if (senalPerdida) {
        comandosDron.throttle = 1000;
        comandosDron.roll = 1500; comandosDron.pitch = 1500; comandosDron.yaw = 1500;
        comandosDron.armable = 0;
    }

    // --- Reset de emergencia: tierra debe desarmar y volver a armar ---
    if (comandosDron.armable == 0) emergencia = false;

    // --- Leer IMU y estimar actitud ---
    float yawRateMeas = 0;
    if (mpuOK) {
        sensors_event_t acc, gyr, temp;
        mpu.getEvent(&acc, &gyr, &temp);

        float accelRoll  = atan2(acc.acceleration.y, acc.acceleration.z);
        float accelPitch = atan2(-acc.acceleration.x,
                                  sqrt(acc.acceleration.y * acc.acceleration.y +
                                       acc.acceleration.z * acc.acceleration.z));
        rollEst  = ALPHA * (rollEst  + gyr.gyro.x * dt) + (1 - ALPHA) * accelRoll;
        pitchEst = ALPHA * (pitchEst + gyr.gyro.y * dt) + (1 - ALPHA) * accelPitch;
        yawRateMeas = gyr.gyro.z;

        // --- Corte de seguridad por inclinacion extrema ---
        if (fabs(rollEst) > DANGER_ANGLE_RAD || fabs(pitchEst) > DANGER_ANGLE_RAD) {
            contadorPeligro++;
            if (contadorPeligro >= DANGER_DEBOUNCE) {
                emergencia = true;
                Serial.println("¡PELIGRO! Inclinacion extrema detectada -> motores apagados.");
            }
        } else {
            contadorPeligro = 0;
        }
    }

    // --- Setpoints desde el mando ---
    float thrPct, rollRef, pitchRef, yawRateRef;
    bool armadoCmd;
    leerSetpoints(thrPct, rollRef, pitchRef, yawRateRef, armadoCmd);

    bool armado = armadoCmd && !senalPerdida && !emergencia && mpuOK;
    if (!armado) { pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); }

    if (armado) {
        // --- PID de actitud ---
        float tauX = pidRoll.update(rollRef, rollEst, dt);
        float tauY = pidPitch.update(pitchRef, pitchEst, dt);
        float tauZ = pidYaw.update(yawRateRef, yawRateMeas, dt);

        // --- Throttle a Newtons + mezcladora + saturacion ---
        float Ftotal = thrPct * (4.0f * TMAX / 100.0f);
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

        aro.fill(aro.Color(0, 150, 255)); // azul = volando/armado

        if (millis() - ultimoDebug > 100) {
            ultimoDebug = millis();
            Serial.printf("ARM:1 roll:%.1f pitch:%.1f (deg) | T1:%d T2:%d T3:%d T4:%d us\n",
                rollEst * RAD_TO_DEG, pitchEst * RAD_TO_DEG,
                pulso[0], pulso[1], pulso[2], pulso[3]);
        }
    } else {
        apagarMotores();
        if (emergencia)        aro.fill(aro.Color(255, 0, 0));   // rojo = emergencia
        else if (senalPerdida) aro.fill(aro.Color(255, 60, 0));  // naranja = failsafe
        else if (!mpuOK)       aro.fill(aro.Color(255, 0, 255)); // magenta = sin IMU, no puede armar
        else                   aro.fill(aro.Color(0, 40, 0));    // verde tenue = desarmado, ok

        if (millis() - ultimoDebug > 100) {
            ultimoDebug = millis();
            Serial.printf("ARM:0 FS:%d EMG:%d MPU:%d | roll:%.1f pitch:%.1f (deg)\n",
                senalPerdida, emergencia, mpuOK, rollEst * RAD_TO_DEG, pitchEst * RAD_TO_DEG);
        }
    }
    aro.show();

    delay(4); // ~200-250 Hz nominal (el ESC de 50Hz limita la actuacion real, no el calculo)
}
