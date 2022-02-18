#include <EEPROM.h>
byte motorNr;

void prepareEEPROM(byte motorNumber)
{
  switch (motorNumber)
  {
    case 1:
      EEPROM.write(0,  11);             //  I2C slave id
      EEPROM.write(1,   1);             //  byte motorId;
      EEPROM.write(2,  A1);             //  byte motorTrackingSensorPin;
      EEPROM.write(3,   3);             //  byte motorSpeedPin;
      EEPROM.write(4,   4);             //  byte motorPin1;
      EEPROM.write(5,   5);             //  byte motorPin2;
      EEPROM.write(6,  16);            //  stepCount           
      break;
    case 2:
      EEPROM.write(0,  12);             //  I2C slave id
      EEPROM.write(1,   2);             //  byte motorId;
      EEPROM.write(2,  A1);             //  byte motorTrackingSensorPin;
      EEPROM.write(3,   3);             //  byte motorSpeedPin;
      EEPROM.write(4,   4);             //  byte motorPin1;
      EEPROM.write(5,   5);             //  byte motorPin2;
      EEPROM.write(6,  16);            //  stepCount           
      break;
    case 3:
      EEPROM.write(0,  13);             //  I2C slave id
      EEPROM.write(1,   3);             //  byte motorId;
      EEPROM.write(2,  A1);             //  byte motorTrackingSensorPin;
      EEPROM.write(3,   3);             //  byte motorSpeedPin;
      EEPROM.write(4,   4);             //  byte motorPin1;
      EEPROM.write(5,   5);             //  byte motorPin2;
      EEPROM.write(6,  16);            //  stepCount           
      break;
    case 4:
      EEPROM.write(10,  11);            //  I2C slave id
      EEPROM.write(11,   4);            //  byte motorId;
      EEPROM.write(12,  A3);            //  byte motorTrackingSensorPin;
      EEPROM.write(13,   9);            //  byte motorSpeedPin;
      EEPROM.write(14,   7);            //  byte motorPin1;
      EEPROM.write(15,   8);            //  byte motorPin2;
      EEPROM.write(16,  16);            //  stepCount           
      break;
    case 5:
      EEPROM.write(10,  12);            //  I2C slave id
      EEPROM.write(11,   5);            //  byte motorId;
      EEPROM.write(12,  A3);            //  byte motorTrackingSensorPin;
      EEPROM.write(13,   9);            //  byte motorSpeedPin;
      EEPROM.write(14,   7);            //  byte motorPin1;
      EEPROM.write(15,   8);            //  byte motorPin2;
      EEPROM.write(16,  16);            //  stepCount           
      break;
    case 6:
      EEPROM.write(10,  13);            //  I2C slave id
      EEPROM.write(11,   6);            //  byte motorId;
      EEPROM.write(12,  A3);            //  byte motorTrackingSensorPin;
      EEPROM.write(13,   9);            //  byte motor;
      EEPROM.write(14,   7);            //  byte motorPin1;
      EEPROM.write(15,   8);            //  byte motorPin2;
      EEPROM.write(16,  16);            //  stepCount           
      break;
  }
}
void setup() 
{

}

void loop() 
{
    Serial.begin(9600);
  Serial.println("Enter motor number (1-6)");
  while (!Serial.available())
  {
        motorNr = (byte) Serial.parseInt();
  }
  prepareEEPROM(motorNr);
  Serial.println("Done");
while (1){};
}
