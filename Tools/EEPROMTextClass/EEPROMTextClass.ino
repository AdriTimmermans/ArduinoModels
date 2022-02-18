#include "Wire.h"
#include "EEPROMTEXT.h"

EEPROMText text;

void setup() {
  // put your setup code here, to run once:
  char logLineBuffer[80]; // used for printing to SD card

  char * lcdLine;
  Wire.begin();
  Serial.begin(9600);
  text.begin(0x52);
  lcdLine = text.readBlock(12, 16, false);
  Serial.print (lcdLine);
  Serial.println("-");

  lcdLine = text.readBlock(57, 32, false);

  Serial.print (lcdLine);
  Serial.println("-");  
  sprintf(logLineBuffer, lcdLine, 103, 16, 1, 24);
  Serial.print (logLineBuffer);
  Serial.println("-");  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
