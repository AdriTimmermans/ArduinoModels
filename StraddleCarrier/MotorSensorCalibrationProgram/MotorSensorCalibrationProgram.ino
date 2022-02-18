#define motorOne                       8
#define motorTwo                       9

#define motorOnePin1                  26
#define motorOnePin2                  27
#define motorTwoPin1                  28
#define motorTwoPin2                  29

#define potMeterOne                   A0
#define sensorMotorOne                A2
#define sensorMotorTwo                A3

#include <EEPROM.h>

int countSensorMotorOne, countSensorMotorTwo;
int sensorValueMotorOne, sensorValueMotorTwo;
int previousSensorValueMotorOne, previousSensorValueMotorTwo;
int potMeterOneValue, previousPotMeterOneValue;
int speedOneValue, speedTwoValue;
int stepSize;

long timeStampBeginIntervalMotorOne, timeStampEndIntervalMotorOne;
long timeStampBeginIntervalMotorTwo, timeStampEndIntervalMotorTwo;

boolean inSync = false;
float frequencyOne, frequencyTwo;

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(motorOne, OUTPUT);
  pinMode(motorTwo, OUTPUT);

  pinMode(motorOnePin1, OUTPUT);
  pinMode(motorOnePin2, OUTPUT);
  pinMode(motorTwoPin1, OUTPUT);
  pinMode(motorTwoPin2, OUTPUT);

  pinMode(potMeterOne, INPUT);
  pinMode(sensorMotorOne, INPUT);
  pinMode(sensorMotorTwo, INPUT);

  speedOneValue = 255; 
  speedTwoValue = 255; 
  digitalWrite(motorOnePin1, HIGH);
  digitalWrite(motorOnePin2, LOW);
  analogWrite(motorOne, speedOneValue);
  digitalWrite(motorTwoPin1, HIGH);
  digitalWrite(motorTwoPin2, LOW);
  analogWrite(motorTwo, speedTwoValue);

}

void loop() {

  sensorValueMotorOne = analogRead(sensorMotorOne);
  sensorValueMotorTwo = analogRead(sensorMotorTwo);
  Serial.print("Sensor value motor One: ");
  Serial.print(sensorValueMotorOne);
  Serial.print(", sensor value motor Two: ");
  Serial.println(sensorValueMotorTwo);
  delay(10);
}
