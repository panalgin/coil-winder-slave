#include <Arduino.h>
#include <SoftwareSerial.h>

#include "MotorController.h"

Motor mainMotor(8, 5, 'X', 8);
Motor vargelMotor(9, 10, 'Y', 16);

MotorController controller;

SoftwareSerial com(12, 11);

void parseMessage(String* message);

void setup()
{
  //(com.begin(115200);

  Serial.begin(115200);

  controller.Initialize();

  vargelMotor.SetLimitSwitches(A2, A3);

  controller.Motors[0] = &mainMotor;
  controller.Motors[1] = &vargelMotor;
}

String incomingMessage = "";
void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();

    if (c != '\n')
      incomingMessage += c;
    else
      parseMessage(&incomingMessage);
  }
}

void parseMessage(String *message)
{
  message->replace("\r", "");

  if (message->startsWith("Offset"))
    controller.Offset();
  else if (message->startsWith("Left"))
    controller.Move('Y', -100);
  else if (message->startsWith("Right"))
    controller.Move('Y', 100);

  *message = "";
}