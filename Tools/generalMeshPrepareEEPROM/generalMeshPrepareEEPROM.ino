#include <EEPROM.h>
byte ID;

void prepareEEPROM(byte equipmentId)
{
  // eeprom 0 = this equipment ID (See ASCCommunicationDefinition.h
  // eeprom 1 - xx = ID of equipment that will communicate


  for (int i = 0; i < 50; i++)
  {
    EEPROM.write(i, 0); // set first 100 bytes to 0
    EEPROM.commit();
  }

  EEPROM.write(0, equipmentId); // set equipID in first byte
  EEPROM.commit();

  switch (equipmentId)
  {
    case 0: // GS(0) <-> S011 (1), T011(5), Z001(11)
      // set for the next 13 bytes where the keep alive message are going to
      Serial.println("EquipmentId: 0");
      EEPROM.write(2, 1);  // to S011
      EEPROM.commit();
      EEPROM.write(6, 1);  // to T011
      EEPROM.commit();
      EEPROM.write(12, 1); // to Z001
      EEPROM.commit();
      break;
    case 1: // S011(1) <-> GS(0), Z001(11)
      Serial.println("EquipmentId: 1");
      EEPROM.commit();
      EEPROM.write(1, 1);  // to GS
      EEPROM.commit();
      EEPROM.write(12, 1);  // to Z001
      EEPROM.commit();
      break;
    case 5: // T011(5) <-> GS(0), Z001(11)
      Serial.println("EquipmentId: 5");
      EEPROM.write(1, 1);  // to GS
      EEPROM.commit();
      EEPROM.write(12, 1);  // to Z001
      EEPROM.commit();
      break;
    case 11: // Z001(11) <-> GS(0), S011(1), T011(5)
      Serial.println("EquipmentId: 11");
      EEPROM.write(1, 1);  //  GS
      EEPROM.commit();
      EEPROM.write(2, 1);  //  S011
      EEPROM.commit();
      EEPROM.write(6, 1);  //  T011
      EEPROM.commit();
      break;
    default:
      Serial.print("Error for equipmentId: ");
      Serial.println(equipmentId);
      break;
  }
}
void setup()
{
  Serial.begin(74880);
  Serial.println ("0=GS00");
  Serial.println ("1=S011");
  Serial.println ("2=S012");
  Serial.println ("3=A011");
  Serial.println ("4=A012");
  Serial.println ("5=T011");
  Serial.println ("6=T012");
  Serial.println ("7=L011");
  Serial.println ("8=L012");
  Serial.println ("9=C011");
  Serial.println ("10=C012");
  Serial.println ("11=Z001");
  Serial.println ("12=Z002");

  Serial.print("Enter equipment nummer :");
  while (!Serial.available())
  {
    ID = (byte) Serial.parseInt();
  }

  EEPROM.begin(50);
  Serial.println (ID);
  prepareEEPROM(ID);
  Serial.println ("EEPROM content");
  for (int i = 0; i < 50; i++)
  {
    Serial.print((int)EEPROM.read(i));
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Done");

}

void loop()
{

}
