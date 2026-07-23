// =========================================================================
// Daltonics_Aire.ino — Firmware de vuelo MANUAL (ESP32, unidad AIRE)
//
// Basado en el sketch de prueba de banco que escribio el usuario (mismos
// pines, desfases de motor, MPU6050 y NeoPixel), pero reemplazando la
// secuencia automatica fija (subir-mantener-bajar) por RECEPCION EN VIVO
// del mando via ESP-NOW: el dron ahora obedece throttle/roll/pitch/yaw
// que manda CONTROL/Enviar_Mando_ESP32_Tierra.m a traves del ESP32 de
// Tierra (ESP32_PRUEBATIERRA.ino, sin cambios).
//
// *** Esto es control MANUAL (mezcla directa mando -> motores), SIN PID ***
// El MPU6050 aqui solo se usa para el corte de seguridad por inclinacion
// extrema, no corrige el vuelo activamente. Eso es la fase siguiente
// (ver Firmware/KE88_MK2_Aire/KE88_MK2_Aire.ino como referencia de PID).
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
    uint16_t yaw;        // 1000-2000 us, centro 1500
    uint8_t  armable;    // 0 = desarmado, 1 = armado
};

PaqueteControl comandosDron = {1000, 1500, 1500, 1500, 0};
unsigned long ultimaVezRecibido = 0;
const unsigned long TIMEOUT_FAILSAFE = 500; // ms sin señal -> failsafe

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
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

// ================= MEZCLADORA MANUAL (sin PID) =================
// Cuanto del desplazamiento del stick (en us, ya viene +-500 desde el
// mando) se traduce en diferencia de PWM entre motores. Empezar bajo y
// subir poco a poco en banco (SIN HELICES primero) - punto de partida
// conservador, TODO ajustar a ojo/sensacion de vuelo.
float K_ROLL  = 0.35f;
float K_PITCH = 0.35f;
float K_YAW   = 0.35f;

const int PWM_MIN_US = 1000;
const int PWM_MAX_US = 2000;

// ================= CORTE DE SEGURIDAD POR INCLINACION =================
// Igual heuristica que el sketch original (accel horizontal ~ g*sin(angulo)),
// pero el umbral se subio de 7.0 (~45 grados) a 9.0 (~67 grados): con
// control manual el piloto SI puede inclinar el dron a proposito dentro
// de rangos normales, asi que el corte automatico debe reservarse para
// perdida de control real (casi volteado), no para maniobras normales.
const float DANGER_ACCEL = 9.0f;
const int   DANGER_DEBOUNCE = 10; // lecturas seguidas (~100ms a 10ms/lectura)
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
void setup() {
    Serial.begin(115200);
    delay(500);

    // --- ESP-NOW primero, para que empiece a recibir cuanto antes ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error critico: no se pudo inicializar ESP-NOW");
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

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

    Serial.println("Iniciando secuencia de armado...");
    animacionArmado(5000);
    Serial.println("Listo. Esperando mando desde Tierra (ESP-NOW)...");
}

// ---------------------------------------------------------------------
unsigned long ultimoDebug = 0;

void loop() {
    // --- Failsafe: sin señal por mas de 500ms -> estado seguro ---
    bool senalPerdida = (millis() - ultimaVezRecibido) > TIMEOUT_FAILSAFE;
    if (senalPerdida) {
        comandosDron.throttle = 1000;
        comandosDron.roll = 1500; comandosDron.pitch = 1500; comandosDron.yaw = 1500;
        comandosDron.armable = 0;
    }

    // --- Reset de emergencia: tierra debe desarmar y volver a armar ---
    if (comandosDron.armable == 0) emergencia = false;

    // --- Lectura de inclinacion (solo para el corte de seguridad) ---
    if (mpuOK) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        if (fabs(a.acceleration.x) > DANGER_ACCEL || fabs(a.acceleration.y) > DANGER_ACCEL) {
            contadorPeligro++;
            if (contadorPeligro >= DANGER_DEBOUNCE) {
                emergencia = true;
                Serial.println("¡PELIGRO! Inclinacion extrema detectada -> motores apagados.");
            }
        } else {
            contadorPeligro = 0;
        }
    }

    bool armado = (comandosDron.armable == 1) && !senalPerdida && !emergencia;

    if (armado) {
        int base = comandosDron.throttle;
        int rollTerm  = (int)((int)comandosDron.roll  - 1500) * K_ROLL;
        int pitchTerm = (int)((int)comandosDron.pitch - 1500) * K_PITCH;
        int yawTerm   = (int)((int)comandosDron.yaw   - 1500) * K_YAW;

        // Mezcladora manual (mismo criterio de signos que la mezcladora
        // del gemelo digital DALTONICS, ver DALTONICS_parametros.m, para
        // el orden fisico M1=FL M2=FR M3=RL M4=RR):
        int m1 = base + rollTerm - pitchTerm - yawTerm; // delantero-izq
        int m2 = base - rollTerm - pitchTerm + yawTerm; // delantero-der
        int m3 = base + rollTerm + pitchTerm + yawTerm; // trasero-izq
        int m4 = base - rollTerm + pitchTerm - yawTerm; // trasero-der
        aplicarPWM(m1, m2, m3, m4);

        aro.fill(aro.Color(0, 150, 255)); // azul = volando/armado
    } else {
        apagarMotores();
        if (emergencia)       aro.fill(aro.Color(255, 0, 0));   // rojo = emergencia
        else if (senalPerdida) aro.fill(aro.Color(255, 60, 0));  // naranja = failsafe
        else                   aro.fill(aro.Color(0, 40, 0));    // verde tenue = desarmado, ok
    }
    aro.show();

    if (millis() - ultimoDebug > 100) {
        ultimoDebug = millis();
        Serial.printf("ARM:%d FS:%d EMG:%d | T:%4d R:%4d P:%4d Y:%4d\n",
            armado, senalPerdida, emergencia,
            comandosDron.throttle, comandosDron.roll, comandosDron.pitch, comandosDron.yaw);
    }

    delay(10); // ~100 Hz
}
