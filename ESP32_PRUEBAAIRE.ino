#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Estructura de datos idéntica para recibir los comandos
struct __attribute__((packed)) PaqueteControl {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint8_t armable;
};

// Variable global donde la controladora de vuelo leerá los comandos del stick
PaqueteControl comandosDron;

// Variable para el Failsafe (Mecanismo de seguridad si se pierde la señal)
unsigned long ultimaVezRecibido = 0;
const unsigned long TIMEOUT_FAILSAFE = 500; // 500ms sin señal = Apagar motores

// Función Callback: Se ejecuta automáticamente por hardware al recibir un paquete
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    // Validar que el tamaño del paquete recibido coincida con nuestra estructura
    if (len == sizeof(PaqueteControl)) {
        memcpy(&comandosDron, incomingData, sizeof(comandosDron));
        ultimaVezRecibido = millis(); // Actualizar el reloj del Failsafe
    }
}

void setup() {
    // Inicializar puerto serie nativo del C3
    Serial.begin(115200);
    delay(2000); // Tiempo para que el puerto USB de la PC reconozca el chip
    
    // Configurar Wi-Fi en modo Estación
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Inicializar ESP-NOW en el aire
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error crítico: No se pudo inicializar ESP-NOW en el Receptor");
        return;
    }

    // Registrar la función callback de recepción
    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
    
    Serial.println("Receptor ESP32-C3 listo y escuchando en el aire...");
}

void loop() {
    // -----------------------------------------------------------------
    // CONTROL DE SEGURIDAD: MÓDULO FAILSAFE
    // -----------------------------------------------------------------
    if (millis() - ultimaVezRecibido > TIMEOUT_FAILSAFE) {
        // Si pasa más de medio segundo sin recibir paquetes del control de Xbox,
        // forzamos valores mínimos de seguridad para evitar que el dron salga volando solo.
        comandosDron.throttle = 1000;
        comandosDron.roll = 1500;
        comandosDron.pitch = 1500;
        comandosDron.yaw = 1500;
        comandosDron.armable = 0; // Desarmar el dron inmediatamente
        
        Serial.println("⚠️ ¡FAILSAFE ACTIVADO! - SEÑAL DEL MANDOO PERDIDA ⚠️");
    } else {
        // Si la señal es buena, imprimimos los datos en el monitor serie para validar en el banco
        Serial.printf("AIRE -> T: %4d | R: %4d | P: %4d | Y: %4d | ARM: %d\n", 
                      comandosDron.throttle, 
                      comandosDron.roll, 
                      comandosDron.pitch, 
                      comandosDron.yaw, 
                      comandosDron.armable);
    }

    // Aquí correrá el bucle principal de tu Flight Controller (Lectura de IMU MPU6050, PID, y PWM a motores)
    delay(20); // Ejecución a 50Hz (Ciclo suave para visualización en terminal)
}