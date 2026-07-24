// =========================================================================
// Calibrar_ESCs.ino — Calibracion de los 4 ESCs del Dron Daltonics
//
// Por que hace falta: cada ESC puede traer de fabrica un punto de arranque
// distinto. Si no se calibran juntos, un mismo comando de throttle produce
// empuje distinto en cada motor -> arranque desincronizado y vuelo
// desbalanceado (justo lo que reportaron: un motor enciende primero, y el
// dron se va siempre hacia el mismo lado).
//
// *** SIN HELICES. SIEMPRE. Este sketch manda pulso MAXIMO a los 4 ESCs. ***
//
// Procedimiento:
//   1. Sin helices puestas, sube este sketch (el ESP32 aun sin bateria de
//      los ESCs, solo por USB).
//   2. Abre el Monitor Serie a 115200 baudios.
//   3. Sigue las instrucciones que va imprimiendo el propio sketch.
// =========================================================================

#include <ESP32Servo.h>

Servo motor1, motor2, motor3, motor4;
const int pinM1 = 1, pinM2 = 2, pinM3 = 3, pinM4 = 4;

void setAll(int us) {
    motor1.writeMicroseconds(us);
    motor2.writeMicroseconds(us);
    motor3.writeMicroseconds(us);
    motor4.writeMicroseconds(us);
}

bool minEnviado = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    motor1.attach(pinM1, 1000, 2000);
    motor2.attach(pinM2, 1000, 2000);
    motor3.attach(pinM3, 1000, 2000);
    motor4.attach(pinM4, 1000, 2000);

    setAll(2000); // pulso MAXIMO desde ya, antes de que conecten la bateria

    Serial.println("=== CALIBRACION DE ESCs - Dron Daltonics ===");
    Serial.println("*** CONFIRMA QUE NO HAY HELICES PUESTAS ANTES DE SEGUIR ***");
    Serial.println();
    Serial.println("1) Este sketch ya esta mandando el pulso MAXIMO (2000us) a los 4 ESCs.");
    Serial.println("2) Conecta la bateria/ESCs AHORA.");
    Serial.println("3) Deberias escuchar un patron de beeps de los 4 ESCs confirmando el maximo.");
    Serial.println("4) Cuando terminen los beeps, escribe cualquier caracter aqui y da ENTER");
    Serial.println("   para mandar el pulso MINIMO (1000us).");
}

void loop() {
    if (!minEnviado) {
        setAll(2000); // seguir mandando maximo mientras esperamos al usuario
        if (Serial.available()) {
            while (Serial.available()) Serial.read(); // limpiar buffer
            minEnviado = true;
            setAll(1000);
            Serial.println();
            Serial.println("Mandando pulso MINIMO (1000us)...");
            Serial.println("Deberias escuchar otro patron de beeps confirmando minimo,");
            Serial.println("seguido del tono normal de 'listo/armado'.");
            Serial.println();
            Serial.println("Calibracion terminada. El pulso se queda fijo en MINIMO");
            Serial.println("(los motores no deben girar en ese punto). Puedes desconectar");
            Serial.println("la bateria cuando quieras y subir el firmware de vuelo.");
        }
    } else {
        setAll(1000); // quedarse en minimo, no hacer nada mas
    }
    delay(50);
}
