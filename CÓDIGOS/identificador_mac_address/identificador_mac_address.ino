#include "WiFi.h"

void setup(){
  Serial.begin(115200);
  delay(2000); // Damos 2 segundos para asegurar estabilidad
  
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(); // <-- Esto fuerza la inicialización del radio
  
  delay(1000); // Esperamos un segundo a que el radio procese
  
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop(){}