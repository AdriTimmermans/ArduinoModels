/*
  External EEPROM Recording & Playback Demo
  ext_eeprom_demo.ino
  Uses AT24LC256 External I2C EEPROM

  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/

// Include the I2C Wire Library
#include "Wire.h"

// EEPROM I2C Address
#define EEPROM_I2C_ADDRESS 0x50

int blockNumber;
String inputLine;
char textBlock[33];
byte dummy;

// Function to write to EEPROM
void writeEEPROM(int address, byte val, int i2c_address)
{
  byte MSB, LSB;
  // Begin transmission to I2C EEPROM
  Wire.beginTransmission(i2c_address);

  // Send memory address as two 8-bit bytes
  MSB = (address & 0xFF00) >> 8;
  LSB = (address & 0x00FF);
  Serial.print ("(");
  Serial.print (MSB);
  Serial.print (",");
  Serial.print (LSB);
  Serial.print ("):" );

  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Serial.print("\t");
  // Send data to be stored
  Wire.write(val);
  Serial.print(val);
  Serial.print("\t");
  Serial.println(char(val));
  // End the transmission
  Wire.endTransmission();

  // Add 5ms delay for EEPROM
  delay(5);
}



void writeBlock (int blockNumber, char textBlock[])
{
    unsigned int eePROMAddress;
    eePROMAddress = (blockNumber - 1) * 32 + 1;
    for (int i = 0; i < 32; i++)
    {
    (textBlock[i] == 0)?writeEEPROM(eePROMAddress, 0x20, EEPROM_I2C_ADDRESS):writeEEPROM(eePROMAddress, textBlock[i], EEPROM_I2C_ADDRESS);
    eePROMAddress++;
  }

}

// Function to read from EEPROM
byte readEEPROM(int address, int i2c_address)
{
  // Define byte for received data
  byte rcvData = 0xFF;

  // Begin transmission to I2C EEPROM
  Wire.beginTransmission(i2c_address);

  // Send memory address as two 8-bit bytes
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB

  // End the transmission
  Wire.endTransmission();

  // Request one byte of data at current memory address
  Wire.requestFrom(i2c_address, 1);

  // Read the data and assign to variable
  rcvData =  Wire.read();

  // Return the data as function output
  return rcvData;
}


void setup()
{

  byte oneChar;
  byte stringLength;
  unsigned int eePROMAddress;
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("initialization done.");

  // Connect to I2C bus as master
  Wire.begin();
}

void loop()
{
  Serial.println("Enter textblock number: ");
  while (Serial.available()== 0){};
  blockNumber = Serial.parseInt();
  Serial.print("textblock number entered: ");
  Serial.println(blockNumber);
  Serial.print("Number of chars still in buffer: ");
  Serial.print(Serial.available());
  while (!Serial.available()== 0)
  { 
    dummy = Serial.read(); 
    Serial.print("-Hex value:");
    Serial.print(dummy, HEX);
  };
  Serial.println("");
  while (Serial.available()== 0){};
  inputLine = Serial.readString();
  Serial.println(inputLine);
  memset(textBlock, 0, sizeof(textBlock));
  inputLine.toCharArray(textBlock, 32);
  for (int i=0;i<=32;i++)
  {
    if (textBlock[i] == 10)
    {
      textBlock[i] = 32;
    }
  }
  Serial.print("Text read:>>");
  Serial.print(textBlock);
  Serial.println("<<");
  // Nothing in loop
  writeBlock (blockNumber, textBlock);
}
