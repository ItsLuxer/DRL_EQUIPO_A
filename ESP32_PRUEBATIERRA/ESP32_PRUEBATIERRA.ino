#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// =========================================================================
// DIRECCIÓN MAC DEL RECEPTOR (Reemplaza con la MAC de tu ESP32-C3 Supermini)
// =========================================================================
uint8_t macReceptor[] = {0x70, 0xAF, 0x09, 0x15, 0x43, 0x2C};

// Estructura de datos para el envío
struct __attribute__((packed)) PaqueteControl {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint8_t armable;
};

PaqueteControl datosAEnviar;
esp_now_peer_info_t peerInfo;

// Callback opcional para verificar si el paquete se entregó con éxito en el aire
// NOTA: firma actualizada para nucleos ESP32 basados en ESP-IDF 5 (Arduino
// core 3.x). La firma vieja (const uint8_t *mac_addr, ...) ya no compila.
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // Descomentar para depuración en banco (puede alentar el bucle a frecuencias altas)
    // Serial.print("Estado de envío: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}

void setup() {
    // Inicializar puerto serie para comunicarse con MATLAB
    Serial.begin(115200);
    
    // Configurar Wi-Fi en modo Estación (necesario para activar el radio)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Inicializar el protocolo ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error crítico: No se pudo inicializar ESP-NOW");
        return;
    }

    // Registrar el callback de envío
    esp_now_register_send_cb(onDataSent);
    
    // Registrar el chip del dron (Peer) en la lista de transmisión
    memcpy(peerInfo.peer_addr, macReceptor, 6);
    peerInfo.channel = 0;  // Usa el canal actual de Wi-Fi
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error: No se pudo registrar el receptor ESP32-C3");
        return;
    }
    
    Serial.println("Sistema Emisor Listo. Esperando tramas de MATLAB...");
}

void loop() {
    // Verificar si MATLAB ha enviado datos por el cable USB
    if (Serial.available() > 0) {
        // Leer la trama de texto hasta encontrar el salto de línea
        String trama = Serial.readStringUntil('\n');
        
        // Buscar los identificadores de la trama: "T1000R1500P1500Y1500A0"
        int tIdx = trama.indexOf('T');
        int rIdx = trama.indexOf('R');
        int pIdx = trama.indexOf('P');
        int yIdx = trama.indexOf('Y');
        int aIdx = trama.indexOf('A');
        
        // Si la trama está completa y bien formateada, parsear los datos
        if (tIdx != -1 && rIdx != -1 && pIdx != -1 && yIdx != -1 && aIdx != -1) {
            datosAEnviar.throttle = trama.substring(tIdx + 1, rIdx).toInt();
            datosAEnviar.roll     = trama.substring(rIdx + 1, pIdx).toInt();
            datosAEnviar.pitch    = trama.substring(pIdx + 1, yIdx).toInt();
            datosAEnviar.yaw      = trama.substring(yIdx + 1, aIdx).toInt();
            datosAEnviar.armable  = trama.substring(aIdx + 1).toInt();
            
            // Disparar los bytes de forma inmediata por el aire hacia el dron
            esp_now_send(macReceptor, (uint8_t *) &datosAEnviar, sizeof(datosAEnviar));
        }
    }
}