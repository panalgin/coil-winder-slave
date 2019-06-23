#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Queue.h"
#include "MotorController.h"

typedef struct
{
  char Axis[2];
  long Delta[2];
  uint16_t Speed;
} Parameter;

typedef struct
{
  uint8_t No;
  Parameter Params;
} Gcode;

typedef struct
{
  uint16_t SpirIndex;
  uint16_t SpirTotal;
  uint16_t WorkingCycle;
  bool IsPaused;
  float WireDiameter;
  float TurnsPerLayer;
  uint16_t Speed;
  bool IsAtEnd;
} Job;

Job CJob = Job();
Job *CurrentJob;

Motor mainMotor(8, 5, 'X', 8);
Motor vargelMotor(9, 6, 'Y', 16);

MotorController controller;
Queue<Gcode> Codes(16);

SoftwareSerial com(12, 11);

void parseMessage(String *message);
void loopSteps();
void parseWork(String *message);
String SplitValues(String *data, char seperator, uint8_t index);
void pauseWork();

void setup()
{
  delay(100);
  com.begin(115200);

  while (!com)
  {
    ;
  }
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
    Parameter params = code.Params;

    if (code.No == 1)
    {
      char axis = params.Axis[0];
      long delta = params.Delta[0];
      uint16_t speed = params.Speed;

      controller.Move(axis, (long)(delta > 0 ? INT32_MAX : INT32_MIN), speed);
    }
    else if (code.No == 0)
    {
      char axis = params.Axis[0];
      long delta = params.Delta[0];
      uint16_t speed = params.Speed;

      controller.Move(axis, delta, speed);
    }
    else if (code.No == 2)
    {
      char firstAxis = params.Axis[0];
      char secondAxis = params.Axis[1];

      long firstDelta = params.Delta[0];
      long secondDelta = params.Delta[1];

      uint16_t maxSpeed = params.Speed;

      controller.LinearMove(firstAxis, secondAxis, firstDelta, secondDelta, maxSpeed);
    }
  }

  if (!controller.IsCompleted())
  {
    if (CurrentJob != NULL) {
      if (!CurrentJob->IsPaused)
        controller.Sync();
    }
    else
      controller.Sync();
  }
}

void parseMessage(String *message)
{
  message->replace("\r", "");

  if (message->startsWith("Pause"))
    pauseWork();

  else if (message->startsWith("Offset-First"))
  {
    controller.Offset("First");

    float firstPos = (float)(controller.KarkasBeginsAt / controller.BaseMetricInSteps);
    com.print("OFD: ");
    com.println(firstPos);
  }
  else if (message->startsWith("Offset-Second"))
  {
    controller.Offset("Second");
    Motor *y = controller.Find('Y');
    long delta = y->CurrentPosition * -1;

    float secondPos = (float)(controller.KarkasEndsAt / controller.BaseMetricInSteps);
    com.print("OSD: ");
    com.println(secondPos);

    delay(500);

    Parameter p = {{'Y'}, {delta}, 300};
    Gcode code = {0, p};

    Codes.push(code);
  }

  else if (message->startsWith("Left"))
  {
    Parameter p = {{'Y'}, {-10}, 10};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("Right"))
  {
    Parameter p = {{'Y'}, {10}, 10};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("Stop"))
  {
    Codes.clear();
    controller.Halt();
  }
  else if (message->startsWith("Work: "))
  {
    parseWork(message);
  }

  *message = "";
}

void pauseWork() {
  if (CurrentJob != NULL) {
    CurrentJob->IsPaused = true;
  }
}

void parseWork(String *message)
{
  message->replace("Work: ", "");

  uint16_t currentCycle = SplitValues(message, '|', 0).toInt();
  float wireDiameter = SplitValues(message, '|', 1).toFloat();
  uint16_t speed = (uint16_t)SplitValues(message, '|', 2).toInt();

  float totalGap = (controller.KarkasEndsAt - controller.KarkasBeginsAt) / controller.BaseMetricInSteps;
  float turnsPerLayer = totalGap / wireDiameter;

  if (CurrentJob == NULL)
  {
    CurrentJob = &CJob;
    CurrentJob->IsPaused = false;
    CurrentJob->Speed = speed;
    CurrentJob->IsAtEnd = false;
    CurrentJob->SpirIndex = 0;
    CurrentJob->TurnsPerLayer = turnsPerLayer;
    CurrentJob->WireDiameter = wireDiameter;
    CurrentJob->WorkingCycle = currentCycle;

    long firstDelta = (unsigned long)(mainMotor.StepsPerRev * mainMotor.MicrostepMultiplier) * turnsPerLayer;
    long secondDelta = (totalGap * vargelMotor.BaseMetricInSteps) * (currentCycle % 2 == 0 ? 1.0f : -1.0f);

    Parameter params = {{'X', 'Y'}, {firstDelta, secondDelta}, speed};
    Gcode code = {2, params};

    Codes.push(code);
  }
  else
  {
    CurrentJob->IsPaused = false;
    CurrentJob->Speed = speed;
    CurrentJob->IsAtEnd = false;
    CurrentJob->WorkingCycle = currentCycle;

    vargelMotor.SetSpeed(speed);
    mainMotor.SetSpeed(speed);
  }
}

String SplitValues(String *data, char seperator, uint8_t index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data->length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data->charAt(i) == seperator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data->substring(strIndex[0], strIndex[1]) : "";
};