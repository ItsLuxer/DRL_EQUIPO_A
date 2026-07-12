#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define I2C_SDA 8
#define I2C_SCL 9

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

Adafruit_BME680 bme;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bme.begin(0x77)) {
    Serial.println("Error: No se encontró el BME680 en 0x77");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  BLEDevice::init("Dron_OKARI_BME");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
}

void loop() {
  if (bme.performReading()) {
    // Orden solicitado: Presión, Temperatura, Humedad
    String data = "P:" + String(bme.pressure / 100.0) + "hPa T:" + String(bme.temperature) + "C H:" + String(bme.humidity) + "%";
    
    // Imprimir en PC
    Serial.println(data);
    
    // Enviar a iPhone (nRF Connect)
    if (deviceConnected) {
      pCharacteristic->setValue(data.c_str());
      pCharacteristic->notify();
    }
  }
  delay(2000);
}