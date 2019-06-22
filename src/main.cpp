#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Queue.h"
#include "MotorController.h"

typedef struct
{
  char Axis;
  long Delta;
  uint16_t Speed;
  bool IsFixed;
} Gcode;

Motor mainMotor(8, 5, 'X', 8);
Motor vargelMotor(9, 6, 'Y', 16);

MotorController controller;

Queue<Gcode> Codes(16);

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

  //Gcode code = {'Y', (long)(100.0f * vargelMotor.BaseMetricInSteps), 100, true};
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
  if (Codes.count() > 0 && controller.IsCompleted())
  {
    Gcode code = Codes.pop();

    if (!code.IsFixed)
      controller.Move(code.Axis, (long)(code.Delta > 0 ? INT32_MAX : INT32_MIN), code.Speed);
    else
      controller.Move(code.Axis, code.Delta, code.Speed);
  }

  if (!controller.IsCompleted())
  {
    controller.Sync();
  }
}

void parseMessage(String *message)
{
  message->replace("\r", "");

  if (message->startsWith("Offset-First"))
  {
    controller.Offset("First");
    com.println("Offset-First-Done");
  }
  else if (message->startsWith("Offset-Second"))
  {
    controller.Offset("Second");
    Motor *y = controller.Find('Y');
    long delta = y->CurrentPosition * -1;
    com.println("Offset-Second-Done");

    delay(500);

    controller.Move('Y', delta, 300);
  }

  else if (message->startsWith("Left"))
  {
    Gcode code = {'Y', -10, 10, false};
    Codes.push(code);
  }
  else if (message->startsWith("Right"))
  {
    Gcode code = {'Y', 10, 10, false};
    Codes.push(code);
  }
  else if (message->startsWith("Stop"))
  {
    Codes.clear();
    controller.Halt();
  }

  *message = "";
}