#include <Wire.h>
#include "EEPROMTEXT.h"
EEPROMText textObject (0x50, 53);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  textObject.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
