#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Queue.h"
#include "MotorController.h"

typedef struct
{
  char Axis[2];
  long Delta[2];
  uint16_t Speed[2];
} Parameter;

typedef struct
{
  uint8_t No;
  Parameter Params;
} Gcode;

Motor mainMotor(8, 5, 'X', 8);
Motor vargelMotor(9, 6, 'Y', 16);

MotorController controller;

Queue<Gcode> Codes(16);

SoftwareSerial com(12, 11);

void parseMessage(String *message);
void loopSteps();
void parseWork(String* message);
String SplitValues(String* data, char seperator, uint8_t index);

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
    Parameter params = code.Params;

    if (code.No == 1)
    {
      char axis = params.Axis[0];
      long delta = params.Delta[0];
      uint16_t speed = params.Speed[0];

      controller.Move(axis, (long)(delta > 0 ? INT32_MAX : INT32_MIN), speed);
    }
    else if (code.No == 0)
    {
      char axis = params.Axis[0];
      long delta = params.Delta[0];
      uint16_t speed = params.Speed[0];

      controller.Move(axis, delta, speed);
    }
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

    Parameter p = {{'Y'}, {delta}, {300}};
    Gcode code = {0, p};

    Codes.push(code);
  }

  else if (message->startsWith("Left"))
  {
    Parameter p = {{'Y'}, {-10}, {10}};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("Right"))
  {
    Parameter p = {{'Y'}, {10}, {10}};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("Stop"))
  {
    Codes.clear();
    controller.Halt();
  }
  else if (message->startsWith("Work: ")) {

  }

  *message = "";
}

void parseWork(String* message) {
  message->replace("Work: ", "");

  float wireDiameter = SplitValues(message, '|', 0).toFloat();
  float totalTurns = SplitValues(message, '|', 1).toFloat();

  float totalGap = (controller.KarkasEndsAt - controller.KarkasBeginsAt) / controller.BaseMetricInSteps;
  float turnsPerLayer = totalGap / wireDiameter;

  Serial.print("Tel Capi: ");
  Serial.println(wireDiameter);
  Serial.print("Toplam Spir: ");
  Serial.println(totalTurns);
  Serial.print("Karkas Boslugu: ");
  Serial.println(totalGap);
  Serial.print("Her sira tur: ");
  Serial.println(turnsPerLayer);
}

String SplitValues(String* data, char seperator, uint8_t index) {
		int found = 0;
		int strIndex[] = { 0, -1 };
		int maxIndex = data->length() - 1;

		for (int i = 0; i <= maxIndex && found <= index; i++) {
			if (data->charAt(i) == seperator || i == maxIndex) {
				found++;
				strIndex[0] = strIndex[1] + 1;
				strIndex[1] = (i == maxIndex) ? i + 1 : i;
			}
		}

		return found > index ? data->substring(strIndex[0], strIndex[1]) : "";
	};