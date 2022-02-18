#include <Wire.h>
#include <LSM303.h>
#include <EEPROM.h>

LSM303 compass;
float measurements[25];
byte sampleSize = 6;
float average;
float maxDegrees, minDegrees;
int   minSteps, maxSteps;
int   trueDegrees, rawDegrees;
float previousDegrees, currentDegrees;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};

float differences[360][3];

int PUL = 7; //define Pulse pin
int DIR = 6; //define Direction pin
int ENA = 5; //define Enable Pin
/* Assign a unique ID to this sensor at the same time */


void setup(void) {
#ifndef ESP8266
  while (!Serial)
    ; // will pause Zero, Leonardo, etc until serial console opens
#endif
  float deadReckoning = -100.0;
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Magnetometer Test");
  Serial.println("");
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  /* Enable auto-gain */
  compass.init();
  compass.enableDefault();

  for (int i = 0; i < 1200; i++) //calibration
  {
    for (int j = 0; j < 53; j++)
    {
      digitalWrite(DIR, LOW);
      digitalWrite(ENA, HIGH);
      digitalWrite(PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL, LOW);
      delayMicroseconds(50);
    }
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);
  }

  compass.m_min = running_min;
  compass.m_max = running_max;

  for (int i = 0; i < 360; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      differences[i][j] = 0;
    }
  }
  compass.read();
  previousDegrees = compass.heading();
  deadReckoning = previousDegrees;

  for (int i = 0; i < 7200; i++) //Forward 5000 steps
  {
    for (int j = 0; j < 53; j++)
    {
      digitalWrite(DIR, LOW);
      digitalWrite(ENA, HIGH);
      digitalWrite(PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL, LOW);
      delayMicroseconds(50);
    }
    compass.read();
    currentDegrees = compass.heading();
    average = previousDegrees * 0.90 + currentDegrees * 0.1;

    if ((average - previousDegrees) > 30.0)
    {
      previousDegrees = 360.0;
      deadReckoning = 360.0;
    }
    else
    {
      previousDegrees = average;
    }
    Serial.print(average);
    Serial.print("\t");

    /*if (abs(average - 10) < 5)
      {
      deadReckoning = (int) average;
      }
      if (deadReckoning < 0.0)
      {
      deadReckoning = 360;
      }
      delay(50);
      if ((deadReckoning >= -0.1) && (deadReckoning <= 360.0))
      {
      differences[(int)deadReckoning][0] = differences[(int)deadReckoning][0] + (deadReckoning - average);
      differences[(int)deadReckoning][1] = differences[(int)deadReckoning][1] + 1;
      differences[(int)deadReckoning][2] = differences[(int)deadReckoning][0] / differences[(int)deadReckoning][1];
      if ((average + differences[(int)deadReckoning][2] > 360.0) || (average + differences[(int)deadReckoning][2] < 0))
      {
        differences[(int)deadReckoning][0] = 0.0;
        differences[(int)deadReckoning][1] = 0;
        differences[(int)deadReckoning][2] = 0.0;
      }
      //Serial.print(average + differences[(int)deadReckoning][2]);
      //Serial.print("\t");
      }
      else
      {
      //Serial.print(average);
      //Serial.print("\t");

      }
      */


      Serial.print(deadReckoning);
      Serial.print("\t");
      deadReckoning -= 1.0; // 1.873415;
      //Serial.print(deadReckoning - average);
    Serial.println();
  }
}

void loop(void) {
}
