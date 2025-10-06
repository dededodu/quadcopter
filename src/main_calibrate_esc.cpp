/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) :
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send 0 throttle
 *     5. Several "beep" tones emits, wich means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send 0 to 180 throttle to ESCs to test them
 */

#include <Arduino.h>
#include <Servo.h>

#define ESC1_PIN 5 
#define ESC2_PIN 6 
#define ESC3_PIN 9 
#define ESC4_PIN 10

#define MIN_PULSE 1000
#define MAX_PULSE 2000

Servo esc1, esc2, esc3, esc4;
char data;

void displayInstructions() {
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send MIN_PULSE");
    Serial.println("\t1 : Send MAX_PULSE");
    Serial.println("\t2 : Run test function\n");
}

void test()
{
    for (int i=0; i<=180; i++) {
        Serial.print("Speed = ");
        Serial.println(i);

        esc1.write(i);
        esc2.write(i);
        esc3.write(i);
        esc4.write(i);

        delay(200);
    }

    Serial.println("STOP");
    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);
}

void setup() {
    Serial.begin(115200);

    esc1.attach(ESC1_PIN, MIN_PULSE, MAX_PULSE);
    esc2.attach(ESC2_PIN, MIN_PULSE, MAX_PULSE);
    esc3.attach(ESC3_PIN, MIN_PULSE, MAX_PULSE);
    esc4.attach(ESC4_PIN, MIN_PULSE, MAX_PULSE);

    displayInstructions();
}

void loop() {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // '0'
            case 0x30:
                Serial.print("Sending MIN_PULSE ");
                Serial.println(MIN_PULSE);
                esc1.writeMicroseconds(MIN_PULSE);
                esc2.writeMicroseconds(MIN_PULSE);
                esc3.writeMicroseconds(MIN_PULSE);
                esc4.writeMicroseconds(MIN_PULSE);
                break;

            // '1'
            case 0x31:
                Serial.print("Sending MAX_PULSE ");
                Serial.println(MAX_PULSE);
                esc1.writeMicroseconds(MAX_PULSE);
                esc2.writeMicroseconds(MAX_PULSE);
                esc3.writeMicroseconds(MAX_PULSE);
                esc4.writeMicroseconds(MAX_PULSE);
                break;

            // '2'
            case 0x32:
                Serial.print("Running test in 3");
                delay(1000);
                Serial.print(" 2");
                delay(1000);
                Serial.println(" 1...");
                delay(1000);
                test();
                break;
        }
    }
}