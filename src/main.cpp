#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Queue.h"
#include "MotorController.h"

typedef struct {
  char Axis;
  long Delta;
  uint16_t Speed;
} Gcode;


Motor mainMotor(8, 5, 'X', 8);
Motor vargelMotor(9, 6, 'Y', 16);

MotorController controller;


Queue<Gcode> Codes[16];

SoftwareSerial com(12, 11);

void parseMessage(String *message);
void loopSteps(); 

void setup()
{
  com.begin(115200);

  Serial.begin(115200);

  controller.Initialize();

  vargelMotor.SetLimitSwitches(A2, A3);
  vargelMotor.SetSpeed(400);

  controller.Motors[0] = &mainMotor;
  controller.Motors[1] = &vargelMotor;
}

String incomingMessage = "";

void loop()
{
  if (com.available())
  {
    char c = com.read();

    Serial.print(c);

    if (c != '\n')
      incomingMessage += c;
    else
      parseMessage(&incomingMessage);
  }

  loopSteps();
}

bool supposedToRun = false;
long lastSteps = 0;

void loopSteps()
{
  if (supposedToRun)
  {
    controller.Move('Y', lastSteps);
  }
}

void parseMessage(String *message)
{
  message->replace("\r", "");

  if (message->startsWith("Offset"))
    controller.Offset();
  else if (message->startsWith("Left"))
  {
    supposedToRun = true;
    lastSteps = -10;
    vargelMotor.SetSpeed(50);
    //controller.Move('Y', -100);
  }
  else if (message->startsWith("Right"))
  {
    supposedToRun = true;
    lastSteps = 10;
    vargelMotor.SetSpeed(50);
    //controller.Move('Y', 100);
  }
  else if (message->startsWith("Stop"))
  {
    supposedToRun = false;
    lastSteps = 0;
  }

  *message = "";
}