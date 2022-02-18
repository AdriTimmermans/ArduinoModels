// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#define startupCloseDownSlave         40    // used to synchronise starting of master and slave arduino
#define startupCloseDownMotors        41    // used to synchronise starting of master and ArduinoProMinis
#define connectI2CSlave               42    // used to connect or disconnect I2C wires when Slave is not powered on
#define connectI2CMotors              43 
void setup() {
  pinMode(startupCloseDownSlave, OUTPUT);
  pinMode(startupCloseDownMotors, OUTPUT);
  pinMode(connectI2CSlave, OUTPUT);
  pinMode(connectI2CMotors, OUTPUT);
  digitalWrite(startupCloseDownMotors, HIGH); // power on motors (Arduino Pro Mini)
  digitalWrite(connectI2CMotors, HIGH);
  digitalWrite(startupCloseDownSlave, HIGH); // power on slave
  digitalWrite(connectI2CSlave, HIGH);
  delay(1000);
  digitalWrite(startupCloseDownMotors, LOW); // power on motors (Arduino Pro Mini)
  digitalWrite(connectI2CMotors, LOW);
  digitalWrite(startupCloseDownSlave, LOW); // power on slave
  digitalWrite(connectI2CSlave, LOW);
  delay(1000);
  Wire.begin();
    Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
  Serial.begin(9600);

  while (!Serial); // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void TCA9548A(uint8_t bus)
{
//  Serial.println("Wire.beginTransmission(0x70);");
  Wire.beginTransmission(0x70);
//  Serial.println("passed");
//  Serial.println("Wire.write(1 << bus);");
  Wire.write(1 << bus);
//  Serial.println("passed");
//  Serial.println("Wire.endTransmission();");
  Wire.endTransmission();
//  Serial.println("passed");
}

void loop()
{
  for (byte i=0;i<8;i++){
  loop1(i);
  }
}

void loop1(byte TCAAddress) {
  int nDevices = 0;
    TCA9548A(TCAAddress);
  Serial.print("Scanning loop ");
  Serial.println(TCAAddress);
  

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000); // Wait 5 seconds for next scan
}
