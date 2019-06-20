#include  <Arduino.h>
#include "MotorController.h"


MotorController::MotorController() {

}

void MotorController::Initialize() {

}

void MotorController::Offset() {
    Serial.println("Offset: Basliyor");

    Motor* vargel = this->Motors[1];
    uint8_t state = digitalRead(vargel->MinSwitchPin);
    
    vargel->SetDirection(Backwards);
    unsigned long lastSwitchReadAt = millis();
    
    while(state == HIGH) {
        vargel->Step();

        if (millis() - lastSwitchReadAt > 30) {
            lastSwitchReadAt = millis();
            state = digitalRead(vargel->MinSwitchPin);
        }
    }

    Serial.println(F("Offset: Bitti"));
}