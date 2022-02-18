//#define motorOne                       8
#define motorOne                       9

//#define motorOnePin1                  26
//#define motorOnePin2                  27
#define motorOnePin1                  28
#define motorOnePin2                  29

//#define sensorMotorOne                A2
#define sensorMotorOne                A3

#include <EEPROM.h>

int countSensorMotorOne;
int sensorValueMotorOne;
int previousSensorValueMotorOne;
int speedOneValue;
int engineNumber;

long timeStampBeginIntervalMotorOne, timeStampEndIntervalMotorOne;
long timeDifference;
float frequencyOne;

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  engineNumber = 2;
  pinMode(motorOne, OUTPUT);

  pinMode(motorOnePin1, OUTPUT);
  pinMode(motorOnePin2, OUTPUT);

  pinMode(sensorMotorOne, INPUT);
  frequencyOne=100;
}

void loop() {

  for (int i=255;i>0;i--)
  {
    if (frequencyOne > 2.0)
    {
      speedOneValue = i;
      digitalWrite(motorOnePin1, HIGH);
      digitalWrite(motorOnePin2, LOW);
      analogWrite(motorOne, speedOneValue);
      
      countSensorMotorOne = 0;
      previousSensorValueMotorOne = 0;
      while(countSensorMotorOne < 5)
      {
        sensorValueMotorOne = analogRead(sensorMotorOne);
        if (previousSensorValueMotorOne == 0)
        {
          previousSensorValueMotorOne  = sensorValueMotorOne;
        }
        if ((sensorValueMotorOne > 500) && (previousSensorValueMotorOne < 500))
        {
          countSensorMotorOne++;
        }
        previousSensorValueMotorOne = sensorValueMotorOne;
      }      
      countSensorMotorOne = 0;
      while(countSensorMotorOne < 25)
      {
        sensorValueMotorOne = analogRead(sensorMotorOne);
        if (previousSensorValueMotorOne == 0)
        {
          previousSensorValueMotorOne  = sensorValueMotorOne;
        }
        if (countSensorMotorOne == 1)
        {
          timeStampBeginIntervalMotorOne = millis();      
        }
        if ((sensorValueMotorOne > 500) && (previousSensorValueMotorOne < 500))
        {
          countSensorMotorOne++;
        }
        previousSensorValueMotorOne = sensorValueMotorOne;
      }      
      timeStampEndIntervalMotorOne = millis();
      timeDifference = timeStampEndIntervalMotorOne - timeStampBeginIntervalMotorOne;
      
      Serial.print("Engine : ");
      Serial.print(engineNumber);
      Serial.print(", power: ");
      Serial.print(speedOneValue);
      Serial.print(", signals: ");
      Serial.print(countSensorMotorOne);
      Serial.print(", in: ");
      Serial.print(timeDifference);
      Serial.print(" ms => frequency = ");
      frequencyOne = (60000.0 / timeDifference) * 3.0;
      Serial.println(frequencyOne);
      if (frequencyOne < 3.5)
      {
        frequencyOne = 0.0;
      }
    }
    EEPROM.write(255-i+(engineNumber-1)*256, (int)(frequencyOne+0.4999));
  }
  digitalWrite(motorOnePin1, LOW);
  digitalWrite(motorOnePin2, LOW);
  analogWrite(motorOne, 0);
  Serial.println("==================");
  Serial.println("Done");
  Serial.println("==================");
  while(1);
}
