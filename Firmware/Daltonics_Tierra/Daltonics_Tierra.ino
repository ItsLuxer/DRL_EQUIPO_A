// =========================================================================
// Daltonics_Tierra.ino — Emisor de Tierra CON TELEMETRIA (ESP32, USB)
//
// Evolucion de ESP32_PRUEBATIERRA.ino (que queda como respaldo):
//   - MATLAB -> USB -> este ESP32 -> ESP-NOW -> dron   (igual que antes)
//   - dron -> ESP-NOW -> este ESP32 -> USB -> MATLAB   (NUEVO)
//
// La telemetria se reenvia a MATLAB como una linea de texto:
//   TL,<roll_deg>,<pitch_deg>,<thr_pct>,<modo>,<flags>
// flags: b0=emergencia b1=failsafe b2=aterrizado b3=mpuOK
//
// Pareja del firmware Firmware/Daltonics_Vuelo (Aire) y del mando
// CONTROL/Control_Vuelo_Daltonics.m.
// =========================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// MAC del ESP32-C3 SuperMini del dron (la misma de ESP32_PRUEBATIERRA)
uint8_t macReceptor[] = {0x70, 0xAF, 0x09, 0x15, 0x43, 0x2C};

struct __attribute__((packed)) PaqueteControl {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint8_t  armable;   // modo 0-3
};

struct __attribute__((packed)) PaqueteTelemetria {
    float   rollDeg;
    float   pitchDeg;
    float   thrPct;
    uint8_t modo;
    uint8_t flags;
};

PaqueteControl datosAEnviar;
esp_now_peer_info_t peerInfo;

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // silencioso; descomentar para depurar entregas
}

// Telemetria del dron -> reenviar a MATLAB por USB
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(PaqueteTelemetria)) {
        PaqueteTelemetria tl;
        memcpy(&tl, incomingData, sizeof(tl));
        Serial.printf("TL,%.1f,%.1f,%.1f,%u,%u\n",
                      tl.rollDeg, tl.pitchDeg, tl.thrPct, tl.modo, tl.flags);
    }
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error critico: No se pudo inicializar ESP-NOW");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    memcpy(peerInfo.peer_addr, macReceptor, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Error: No se pudo registrar el receptor ESP32-C3");
        return;
    }

    Serial.println("Tierra con telemetria lista. Esperando tramas de MATLAB...");
}

void loop() {
    if (Serial.available() > 0) {
        String trama = Serial.readStringUntil('\n');

        int tIdx = trama.indexOf('T');
        int rIdx = trama.indexOf('R');
        int pIdx = trama.indexOf('P');
        int yIdx = trama.indexOf('Y');
        int aIdx = trama.indexOf('A');

        if (tIdx != -1 && rIdx != -1 && pIdx != -1 && yIdx != -1 && aIdx != -1) {
            datosAEnviar.throttle = trama.substring(tIdx + 1, rIdx).toInt();
            datosAEnviar.roll     = trama.substring(rIdx + 1, pIdx).toInt();
            datosAEnviar.pitch    = trama.substring(pIdx + 1, yIdx).toInt();
            datosAEnviar.yaw      = trama.substring(yIdx + 1, aIdx).toInt();
            datosAEnviar.armable  = trama.substring(aIdx + 1).toInt();

            esp_now_send(macReceptor, (uint8_t *) &datosAEnviar, sizeof(datosAEnviar));
        }
    }
}
