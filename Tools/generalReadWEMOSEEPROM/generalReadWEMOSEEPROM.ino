#include <EEPROM.h>

void setup()
{
  Serial.begin(74880);
  EEPROM.begin(100);
  Serial.println ("EEPROM content");
  for (int i = 0; i < 99; i++)
  {
    Serial.print(EEPROM.read(i));
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Done");

}

void loop()
{
  Serial.println ("EEPROM content");
  for (int i = 0; i < 99; i++)
  {
    Serial.print(EEPROM.read(i));
    Serial.print(" ");
  }
  Serial.println();
  delay(2000);
}
