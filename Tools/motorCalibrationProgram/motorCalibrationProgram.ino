#define motorRight                       9
#define motorRightPin1                   7
#define motorRightPin2                   8
#define sensorRight                     A3

#define motorLeft                        3
#define motorLeftPin1                    4
#define motorLeftPin2                    5
#define sensorLeft                      A1

#include <EEPROM.h>

int countSensor;
int sensorValue;
int lastSensorValue;
int speedValue;
int stepSize = 3;
int interval = 1000;

long timeStampBeginInterval;
byte motorNr, dummy;

float frequency;

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:

  pinMode(motorRight, OUTPUT);

  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);

  pinMode(sensorRight, INPUT);
  pinMode(motorLeft, OUTPUT);

  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);

  pinMode(sensorLeft, INPUT);
}

void prepareSpeedEEPROM (int motorPin, int motorPin1, int motorPin2, int sensorPin, int startAddress)
{
  int pointer = 0;
  frequency = 300;
  lastSensorValue = 0;
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  analogWrite(motorPin, 255);
/*
  for (int i = 255; i > 0; i = i - stepSize)
  {
    speedValue = i;
    //    if (frequency > 0)
    //    {
    analogWrite(motorPin, speedValue);
    countSensor = 0;
    timeStampBeginInterval = millis();
    while ((millis() - timeStampBeginInterval) < interval)
    {
      sensorValue = analogRead(sensorPin);
      delay(20);
      if (abs(lastSensorValue - sensorValue) > 500)
      {
        countSensor++;
        delay(20);
      }
      lastSensorValue = sensorValue;
    }
    Serial.print(",");
    Serial.print(countSensor);
    frequency = (countSensor * (60000 / interval)) / 16.0;
    //    }

    Serial.print("\t,");
    Serial.print(255 - i + startAddress);

    Serial.print("\t,");
    Serial.print(speedValue);

    Serial.print("\t,");
    Serial.println((int)frequency);
    EEPROM.write(startAddress+pointer, (int)frequency);
    //EEPROM.write(555-i+startAddress, (int)frequency);
    countSensor = 0;
    pointer++;
  }
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  Serial.println("Done");
*/  
}

void loop()
{
  Serial.println("Enter motor number (1-6)");
  while(!Serial.available()){};
  motorNr = (byte)Serial.parseInt();
  Serial.print("Motor selected = ");
  Serial.println(motorNr);
  while (Serial.available())
  {
    dummy = Serial.read();
  }
  if (motorNr > 3)
  {
    prepareSpeedEEPROM (motorRight, motorRightPin1, motorRightPin2, sensorRight, 100);
  }
  else
  {
    prepareSpeedEEPROM (motorLeft, motorLeftPin1, motorLeftPin2, sensorLeft, 400);
  }
  delay(10000);
}
