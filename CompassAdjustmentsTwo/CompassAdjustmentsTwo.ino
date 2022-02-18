#include <Wire.h>
#include <LSM303.h>
#include<ASCModelParameters.h>

#include <EEPROM.h>
#define compassMotorPin1              A8    // 28BYJ48 pin 1
#define compassMotorPin2              A9    // 28BYJ48 pin 2
#define compassMotorPin3             A10     // 28BYJ48 pin 3
#define compassMotorPin4             A11    // 28BYJ48 pin 4
int compassMotorPosition = 0;
const uint8_t mainI2CChannel = 1;

LSM303 compass;
byte sampleSize = 6;
const int stepsPerRevolution = 512;
float average;
float maxDegrees, minDegrees;
int   minSteps, maxSteps;
int   trueDegrees, rawDegrees;
float previousDegrees, currentDegrees;
float uniPolarstepperMotoronedegree = (float)stepsPerRevolution / 360.0;
float uniPolarstepperMotorDegreesPerStep = 360.0 / (float)stepsPerRevolution;
int lookup[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};

LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};

/* Assign a unique ID to this sensor at the same time */

void compassMotorAntiClockWise(int rotateDegrees)
{
  int aux = (float)rotateDegrees * uniPolarstepperMotoronedegree;

  for (int j = 0; j < aux; ++j)
  {
    for (int i = 7; i >= 0; i--)
    {
      setOutputCompass(i);
      delayMicroseconds(compassMotorSpeed);
    }
  }
  digitalWrite(compassMotorPin1, LOW);
  digitalWrite(compassMotorPin2, LOW);
  digitalWrite(compassMotorPin3, LOW);
  digitalWrite(compassMotorPin4, LOW);
  compassMotorPosition = compassMotorPosition - aux;
  //saveCompassMotorPosition ();
}
void setOutputCompass(int out)
{
  (bitRead(lookup[out], 0) == 1) ? digitalWrite(compassMotorPin1, HIGH) : digitalWrite(compassMotorPin1, LOW);
  (bitRead(lookup[out], 1) == 1) ? digitalWrite(compassMotorPin2, HIGH) : digitalWrite(compassMotorPin2, LOW);
  (bitRead(lookup[out], 2) == 1) ? digitalWrite(compassMotorPin3, HIGH) : digitalWrite(compassMotorPin3, LOW);
  (bitRead(lookup[out], 3) == 1) ? digitalWrite(compassMotorPin4, HIGH) : digitalWrite(compassMotorPin4, LOW);
}

void compassMotorClockWise(int rotateDegrees)
{
  int aux = (float)rotateDegrees * uniPolarstepperMotoronedegree;

  for (int j = 0; j < aux; ++j)
  {
    for (int i = 0; i < 8; ++i)
    {
      setOutputCompass(i);
      delayMicroseconds(compassMotorSpeed);
    }
  }
  digitalWrite(compassMotorPin1, LOW);
  digitalWrite(compassMotorPin2, LOW);
  digitalWrite(compassMotorPin3, LOW);
  digitalWrite(compassMotorPin4, LOW);
  compassMotorPosition = compassMotorPosition + aux;
  //saveCompassMotorPosition ();

}

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void setup(void) {

  pinMode(compassMotorPin1, OUTPUT);
  pinMode(compassMotorPin2, OUTPUT);
  pinMode(compassMotorPin3, OUTPUT);
  pinMode(compassMotorPin4, OUTPUT);

  float deadReckoning = -100.0;
  Serial.begin(9600);
  Wire.begin();
  TCA9548A(mainI2CChannel);
  compass.init();
  compass.enableDefault();
  delay(1000);
//  Serial.println("Magnetometer Test");

  for (int j = 0; j < 6; j++)
  {
    for (int i = 0; i < 360; i=i+5) //calibration
    {
      ((j % 2) == 0) ? compassMotorClockWise(5) : compassMotorAntiClockWise(5);
      compass.read();
      delay(50);
      running_min.x = min(running_min.x, compass.m.x);
      running_min.y = min(running_min.y, compass.m.y);
      running_min.z = min(running_min.z, compass.m.z);

      running_max.x = max(running_max.x, compass.m.x);
      running_max.y = max(running_max.y, compass.m.y);
      running_max.z = max(running_max.z, compass.m.z);
/*      Serial.print(i);
      Serial.print("\t min:{");
      Serial.print(running_min.x);
      Serial.print(",");
      Serial.print(running_min.y);
      Serial.print(",");
      Serial.print(running_min.z);
      Serial.print("}, max:{");
      Serial.print(running_max.x);
      Serial.print(",");
      Serial.print(running_max.y);
      Serial.print(",");
      Serial.print(running_max.z);
      Serial.println("}");*/
    }
    //Serial.println();
  }

  compass.m_min = running_min;
  compass.m_max = running_max;

  compass.read();
  previousDegrees = compass.heading();
  deadReckoning = previousDegrees;

  for (int j = 0; j < 6; j++)
  {
    for (int i = 0; i < 360; i=i+5) //Forward 5000 steps
    {
      ((j % 2) == 0) ? compassMotorClockWise(5) : compassMotorAntiClockWise(5);
      compass.read();
      currentDegrees = compass.heading();
//      average = previousDegrees * 0.90 + currentDegrees * 0.1;
      average = currentDegrees;

      if (abs(average - previousDegrees) > 30.0)
      {
        if ((j%2) == 0)
        {
        previousDegrees = 0.0;
        deadReckoning = 0.0;          
        }
        else
        {
        previousDegrees = 360.0;
        deadReckoning = 360.0;
        }
      }
      else
      {
        previousDegrees = average;
      }
      Serial.print(average);
      Serial.print("\t");

      Serial.print(deadReckoning);
      Serial.print("\t");
      
      ((j % 2) == 0) ? deadReckoning += 5.0: deadReckoning -= 5.0;
      // deadReckoning += 1.0; // 1.873415;
      Serial.println();
    }
  }
}
void loop(void)
{
}
