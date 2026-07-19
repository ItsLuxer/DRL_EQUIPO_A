// =========================================================================
// KE88_MK2_Aire.ino — Firmware de vuelo (ESP32, unidad AIRE)
//
// Fase 1 del roadmap: PID embebido corriendo LOCALMENTE en este ESP32
// (no en MATLAB — el enlace ESP-NOW/Serial tiene demasiada latencia para
// un lazo de estabilizacion). MATLAB/el gemelo digital solo diseñan y
// validan las ganancias; aqui se portan a codigo.
//
// Reutiliza el protocolo (struct + failsafe) de ESP32_PRUEBAAIRE.ino para
// seguir siendo compatible con el emisor de tierra ya existente
// (ESP32_PRUEBATIERRA.ino). No modifica esos sketches ni el gemelo
// digital en Dron_Digital/MATLAB_DRIVE.
//
// Estado del proyecto al escribir esto: el chasis MK2 (600-1000 g)
// todavia no esta armado ni el MPU6050 montado, asi que varios valores
// abajo son PLACEHOLDERS marcados con "TODO" — se deben confirmar/medir
// antes de instalar helices.
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>   // Instalar via Library Manager: "ESP32Servo"

// ---------------------------------------------------------------------
// Protocolo de comandos (identico a ESP32_PRUEBAAIRE.ino)
// ---------------------------------------------------------------------
struct __attribute__((packed)) PaqueteControl {
    uint16_t throttle;   // convencion RC: 1000-2000 us
    uint16_t roll;       // 1000-2000 us, centro 1500
    uint16_t pitch;      // 1000-2000 us, centro 1500
    uint16_t yaw;        // 1000-2000 us, centro 1500 (yaw-RATE, no angulo)
    uint8_t  armable;    // 0 = desarmado, 1 = armado
};

PaqueteControl comandosDron;
unsigned long ultimaVezRecibido = 0;
const unsigned long TIMEOUT_FAILSAFE = 500; // ms sin señal -> failsafe

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(PaqueteControl)) {
        memcpy(&comandosDron, incomingData, sizeof(comandosDron));
        ultimaVezRecibido = millis();
    }
}

// ---------------------------------------------------------------------
// TODO (hardware): confirmar placa real. El comentario de
// ESP32_PRUEBAAIRE.ino dice "ESP32-C3", pero mpu650_serial_monitor.ino
// usa GPIO16/17 para I2C, que NO existen en un ESP32-C3. Los pines de
// abajo son los tipicos de un ESP32 devkit de 38 pines; si la placa real
// es un C3 Supermini, hay que cambiarlos (I2C tipico en C3: SDA=8, SCL=9,
// como en bme680_bluetooth.ino).
// ---------------------------------------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22

const int PIN_ESC[4] = {13, 12, 14, 27}; // TODO: confirmar GPIO reales de cada ESC
// Orden de motores (igual que KE88_parametros.m, vista superior, x=adelante, y=izquierda):
//   M1 = delantero-izq (+x,+y)   M2 = delantero-der (+x,-y)
//   M3 = trasero-der   (-x,-y)   M4 = trasero-izq   (-x,+y)

const int PWM_MIN_US = 1000;  // ESC parado / calibracion minima
const int PWM_MAX_US = 2000;  // ESC empuje maximo

// ---------------------------------------------------------------------
// Geometria y empuje del chasis MK2 — TODO: medir en el chasis real.
// Placeholders tomados como referencia de orden de magnitud, NO validos
// para volar. dx/dy se miden centro->motor; Tmax es empuje maximo POR
// MOTOR en Newtons; c es el coeficiente de par de arrastre (ver
// KE88_parametros.m).
// ---------------------------------------------------------------------
float DX   = 0.09f;   // m  TODO: medir
float DY   = 0.09f;   // m  TODO: medir
float TMAX = 3.0f;    // N por motor  TODO: medir/estimar del datasheet del motor+helice
float CYAW = 0.012f;  // coef. par de arrastre  TODO: estimar

// ---------------------------------------------------------------------
// Ganancias PID — PLACEHOLDERS MUY CONSERVADORES a proposito.
// NO son las de KE88_parametros.m (esas son para un dron de 87 g).
// TODO: reemplazar con las ganancias que Control tunee en el gemelo
// digital MK2 (600-1000 g) antes de instalar helices.
// ---------------------------------------------------------------------
float Kp_ang = 0.0005f, Ki_ang = 0.0f,    Kd_ang = 0.0001f, N_filt = 20.0f;
float Kp_yaw = 0.0005f, Ki_yaw = 0.0f;

// ---------------------------------------------------------------------
// PID 2-DOF con derivada sobre la medicion (evita "derivative kick"),
// misma estructura que el bloque "PID Controller (2DOF)" usado en
// KE88_construir_modelo.m para PID_Roll y PID_Pitch (b=1, c=0).
// ---------------------------------------------------------------------
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

// PI simple (yaw-rate), igual que el bloque "PID Controller" (Controller=PI)
// usado en PID_Yaw dentro de KE88_construir_modelo.m.
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

Adafruit_MPU6050 mpu;
Servo escs[4];

// Estimacion de actitud (filtro complementario)
float rollEst = 0, pitchEst = 0;   // rad
const float ALPHA = 0.98f;

// Mezcladora: Mix = inv(A), A construida igual que en KE88_parametros.m
float Mix[4][4];

// ---------------------------------------------------------------------
// Inversion de matriz 4x4 por Gauss-Jordan (para no tener que derivar el
// inverso de A a mano; se calcula una sola vez en setup() a partir de
// DX/DY/CYAW, igual que "P.Mix = inv(A)" en KE88_parametros.m).
// ---------------------------------------------------------------------
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
    // Misma A que KE88_parametros.m (orden de fila: F, tau_x, tau_y, tau_z;
    // orden de columna: T1..T4).
    float A[4][4] = {
        { 1,     1,     1,     1   },
        { DY,   -DY,   -DY,    DY  },
        {-DX,   -DX,    DX,    DX  },
        {-CYAW,  CYAW, -CYAW,  CYAW}
    };
    invertir4x4(A, Mix);
}

// ---------------------------------------------------------------------
// Conversion de PaqueteControl (convencion RC 1000-2000 us) a las mismas
// unidades fisicas que usa KE88_control.slx (thr %, roll/pitch deg,
// yaw-rate dps) para que las ganancias porten 1:1 desde el gemelo digital.
// ---------------------------------------------------------------------
void leerSetpoints(float &thrPct, float &rollRef, float &pitchRef, float &yawRateRef, bool &armado) {
    thrPct     = constrain(map(comandosDron.throttle, 1000, 2000, 0, 100), 0, 100);
    rollRef    = (constrain((int)comandosDron.roll,  1000, 2000) - 1500) / 500.0f * 25.0f  * DEG_TO_RAD;
    pitchRef   = (constrain((int)comandosDron.pitch, 1000, 2000) - 1500) / 500.0f * 25.0f  * DEG_TO_RAD;
    yawRateRef = (constrain((int)comandosDron.yaw,   1000, 2000) - 1500) / 500.0f * 120.0f * DEG_TO_RAD;
    armado     = (comandosDron.armable == 1);
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ---------------------------------------------------------------------
unsigned long ultimoDebug = 0;
unsigned long tPrevControl = 0;
const unsigned long LOOP_US = 4000; // 250 Hz

void setup() {
    Serial.begin(115200);
    delay(500);

    // --- ESP-NOW (identico a ESP32_PRUEBAAIRE.ino) ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error critico: no se pudo inicializar ESP-NOW");
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

    // Estado seguro inicial (igual a los valores de failsafe)
    comandosDron = {1000, 1500, 1500, 1500, 0};

    // --- IMU ---
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        Serial.println("ERROR: MPU6050 no detectado. Revisar cableado/pines I2C.");
    }

    // --- ESCs ---
    for (int i = 0; i < 4; i++) {
        escs[i].setPeriodHertz(50);
        escs[i].attach(PIN_ESC[i], PWM_MIN_US, PWM_MAX_US);
        escs[i].writeMicroseconds(PWM_MIN_US); // motores parados al iniciar
    }

    // --- PID / mezcladora ---
    pidRoll.Kp = Kp_ang;  pidRoll.Ki = Ki_ang;  pidRoll.Kd = Kd_ang;  pidRoll.N = N_filt;
    pidPitch.Kp = Kp_ang; pidPitch.Ki = Ki_ang; pidPitch.Kd = Kd_ang; pidPitch.N = N_filt;
    pidYaw.Kp = Kp_yaw;   pidYaw.Ki = Ki_yaw;
    construirMezcladora();

    tPrevControl = micros();
    Serial.println("KE88_MK2_Aire listo. Motores parados, esperando armar+señal.");
}

void loop() {
    unsigned long ahora = micros();
    if (ahora - tPrevControl < LOOP_US) return;
    float dt = (ahora - tPrevControl) / 1e6f;
    tPrevControl = ahora;

    // --- Failsafe (misma logica de ESP32_PRUEBAAIRE.ino) ---
    bool senalPerdida = (millis() - ultimaVezRecibido) > TIMEOUT_FAILSAFE;
    if (senalPerdida) {
        comandosDron.throttle = 1000;
        comandosDron.roll = 1500; comandosDron.pitch = 1500; comandosDron.yaw = 1500;
        comandosDron.armable = 0;
    }

    // --- Leer IMU y estimar actitud ---
    // TODO: validar signo/orientacion de roll/pitch contra el montaje real
    // del MPU6050 en el chasis; ajustar ejes de ax/ay/az si es necesario.
    sensors_event_t acc, gyr, temp;
    mpu.getEvent(&acc, &gyr, &temp);

    float accelRoll  = atan2(acc.acceleration.y, acc.acceleration.z);
    float accelPitch = atan2(-acc.acceleration.x,
                              sqrt(acc.acceleration.y * acc.acceleration.y +
                                   acc.acceleration.z * acc.acceleration.z));
    rollEst  = ALPHA * (rollEst  + gyr.gyro.x * dt) + (1 - ALPHA) * accelRoll;
    pitchEst = ALPHA * (pitchEst + gyr.gyro.y * dt) + (1 - ALPHA) * accelPitch;
    float yawRateMeas = gyr.gyro.z; // rad/s, usado directo (sin integrar) en el PI de yaw

    // --- Setpoints desde el mando (via ESP-NOW) ---
    float thrPct, rollRef, pitchRef, yawRateRef;
    bool armado;
    leerSetpoints(thrPct, rollRef, pitchRef, yawRateRef, armado);

    if (!armado) { pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); }

    // --- PID de actitud (misma estructura que KE88_dron.slx) ---
    float tauX = pidRoll.update(rollRef, rollEst, dt);      // N*m
    float tauY = pidPitch.update(pitchRef, pitchEst, dt);   // N*m
    float tauZ = pidYaw.update(yawRateRef, yawRateMeas, dt);// N*m

    // --- Throttle a Newtons (igual que el bloque Thr_a_Newtons: 4*Tmax/100) ---
    float Ftotal = thrPct * (4.0f * TMAX / 100.0f);

    // --- Mezcladora + saturacion (igual que Mezcladora + Sat_Motores) ---
    float u[4] = {Ftotal, tauX, tauY, tauZ};
    float T[4];
    for (int i = 0; i < 4; i++) {
        T[i] = Mix[i][0]*u[0] + Mix[i][1]*u[1] + Mix[i][2]*u[2] + Mix[i][3]*u[3];
        T[i] = constrain(T[i], 0.0f, TMAX);
    }

    // --- Salida a ESCs ---
    int pulso[4];
    for (int i = 0; i < 4; i++) {
        pulso[i] = armado ? (int)constrain(mapFloat(T[i], 0.0f, TMAX, PWM_MIN_US, PWM_MAX_US), PWM_MIN_US, PWM_MAX_US)
                           : PWM_MIN_US;
        escs[i].writeMicroseconds(pulso[i]);
    }

    // --- Debug por Serial (~10 Hz) ---
    if (millis() - ultimoDebug > 100) {
        ultimoDebug = millis();
        Serial.printf("ARM:%d FS:%d | roll:%.1f pitch:%.1f (deg) | T1:%d T2:%d T3:%d T4:%d us\n",
            armado, senalPerdida,
            rollEst * RAD_TO_DEG, pitchEst * RAD_TO_DEG,
            pulso[0], pulso[1], pulso[2], pulso[3]);
    }
}
