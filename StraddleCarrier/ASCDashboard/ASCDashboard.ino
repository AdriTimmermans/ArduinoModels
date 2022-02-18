// This is the logging and matrixboard slave id=0x0E
#define DEBUG_PRINT 1

#ifdef DEBUG_PRINT
#define _SERIAL_BEGIN(x) Serial.begin(x);
#define _SERIAL_PRINT(x) Serial.print(x);
#define _SERIAL_PRINT_HEX(x) Serial.print(x, HEX);
#define _SERIAL_PRINTLN(x) Serial.println(x);
#define _SERIAL_PRINTLN_HEX(x) Serial.println(x, HEX);
#else
#define _SERIAL_BEGIN(x)
#define _SERIAL_PRINT(x)
#define _SERIAL_PRINT_HEX(x)
#define _SERIAL_PRINTLN(x)
#define _SERIAL_PRINTLN_HEX(x)
#endif

#include <Wire.h>
#include <SPI.h>
//#include <ASCCommunicationDefinition.h>
#include <SD.h>
//#include <DS3231.h>
#include <LedControl.h>
//DS3231 RTC;
//RTClib defineNow;
#include<EEPROM.h>

enum ASCSlaveCommands {
  requestToLiftSpreader         = 30, // command sent to slave to trigger lifting the spreader
  requestToDropSpreader         = 31, // command sent to slave to trigger dropping the spreader
  requestDisplayLines           = 32, // command sent to slave to get display lines
  requestInterruptType          = 33, // command sent to slave to inform after type of interrupt
  requestSpreaderHeight         = 34, // command sent to slave to inform after the spreader height;
  requestSlaveReady             = 35, // command sent to slave to inform after the readyness of the slave (is it out of setup?);
  informMatrixBoardData         = 37, // command sent to nano to inform that a matrixboard message is coming
  requestResetSlave             = 38, // command sent to slave to execute a reset after the start
  informSlaveToStart            = 39, // command sent to slave to inform slave of master being ready for slave to start
  heartBeat                     = 40, // master still active
  informToReceiveDashboardInfo  = 41, // command to inform slave that dashboard info is coming
  requestWheelPulsesCount       = 42, // request to motor mcu to update number of wheel pulses
  informMotorOrder              = 43, // command with motor order info
  requestDisplayMotorData       = 44, // request for motor data to display
  informExecuteMotorOrder       = 45, // command to execute the motor order
  informLoglinePart1            = 46, // command sent to nano to inform that a logline is coming (chars 0-29)
  informLoglinePart2            = 47, // command sent to nano to inform that a logline is coming (chars 30-59)
  informLoglinePart3            = 48, // command sent to nano to inform that a logline is coming (chars 60-80)
  logLineInterrupt              = 94,
  noMessage                     = 95,
  displayLineInterrupt          = 96,
  obstacleInterrupt             = 97,
  slaveBusy                     = 98,
  slaveReady                    = 99
};

// pindefinitions:
//      I2C SCL                        A5
//      I2C SDA                        A4
#define SDCardChipSelectPin            4    // pin to enable the SDcard 
//      SPI MOSI                       D11
//      SPI MISO                       D12
//      SPI CLK                        D13

volatile boolean logLineReceived = false;
volatile boolean matrixBoardDataReceived = false;
char          lfn[14];
File          myFile;
int count = 0;

boolean loggingOnFile = false;
volatile int masterCommand = 0;
int masterInfoRequest = 0;
volatile byte logLine[80];
volatile byte matrixBoardData[9];

int DIN = 9;
int CLK = 8;
int CS  = 7;
LedControl lc = LedControl(DIN, CLK, CS, 1);

#define THIS_NODE_ADDRESS 0x0E

/*
  void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = defineNow.now();

  // return date using FAT_DATE macro to format fields
   date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
   time = FAT_TIME(now.hour(), now.minute(), now.second());
  }
*/

void(* resetFunc) (void) = 0;//declare reset function at address 0

void receiveEvent(int howMany)
{
  int aux;
  int characterCount;
  _SERIAL_PRINTLN("I2C ");
  aux = Wire.read();
  _SERIAL_PRINT("Msg: ");
  _SERIAL_PRINTLN(aux);
  switch (aux)
  {
    case informLoglinePart1:    // checked
      memset (logLine, 32, 80);
      characterCount = 0;      
      while(Wire.available())
      {
        logLine[characterCount] = Wire.read();
        _SERIAL_PRINT(char(logLine[characterCount]));
        (characterCount <79)?characterCount++:characterCount=79;
      }
      break;

    case informLoglinePart2:    // checked
      characterCount = 30;
      while(Wire.available())
      {
        logLine[characterCount] = Wire.read();
        _SERIAL_PRINT(char(logLine[characterCount]));
        (characterCount <79)?characterCount++:characterCount=79;
      }
      break;

    case informLoglinePart3:    // checked
      characterCount = 60;
      while(Wire.available())
      {
        logLine[characterCount] = Wire.read();
        _SERIAL_PRINT(char(logLine[characterCount]));
        (characterCount <79)?characterCount++:characterCount=79;
      }
      logLine[79] = '\0';
      logLineReceived = true;
      break;

    case informMatrixBoardData:  //checked
      characterCount = 0;
      Serial.println("Matrixbord data:");
      while(Wire.available())
      {
        matrixBoardData[characterCount] = Wire.read();
        Serial.println(matrixBoardData[characterCount], BIN);
        (characterCount <7)?characterCount++:characterCount=7;
      }
      matrixBoardDataReceived = true;
      break;
  }
  _SERIAL_PRINTLN();
}

void setMatrixBoard (byte character [])
{
  for (int i = 0; i < 8; ++i)
  {
    lc.setRow(0, i, character[i]);
  }
}

void dumpLogLine ()
{
  if (loggingOnFile)
  {
    myFile = SD.open(lfn, FILE_WRITE);
    // if the file opened okay, write to it:
    if (myFile)
    {
      _SERIAL_PRINT("To logfile: ");
      for (int i = 0; i < 80; i++)
      {
        if (logLine[i] > 0)
        {
          myFile.print(char(logLine[i]));
          _SERIAL_PRINT (char(logLine[i]));
        }
      }
      myFile.println();
      _SERIAL_PRINTLN("<<");
      myFile.close(); // close the file

    }
    else
    {
      _SERIAL_PRINTLN("Dump logline failed");
    }
  }
}

void writeArduinoOnMatrix(int delaytime) {
  /* here is the data for the characters */
  _SERIAL_PRINTLN("In writeArduinoOnMatrix");
  byte a[5] = {B01111110, B10001000, B10001000, B10001000, B01111110};
  byte r[5] = {B00111110, B00010000, B00100000, B00100000, B00010000};
  byte d[5] = {B00011100, B00100010, B00100010, B00010010, B11111110};
  byte u[5] = {B00111100, B00000010, B00000010, B00000100, B00111110};
  byte i[5] = {B00000000, B00100010, B10111110, B00000010, B00000000};
  byte n[5] = {B00111110, B00010000, B00100000, B00100000, B00011110};
  byte o[5] = {B00011100, B00100010, B00100010, B00100010, B00011100};

  /* now display them one by one with a small delay */

  lc.setRow(0, 0, a[0]);
  lc.setRow(0, 1, a[1]);
  lc.setRow(0, 2, a[2]);
  lc.setRow(0, 3, a[3]);
  lc.setRow(0, 4, a[4]);
  delay(delaytime);
  lc.setRow(0, 0, r[0]);
  lc.setRow(0, 1, r[1]);
  lc.setRow(0, 2, r[2]);
  lc.setRow(0, 3, r[3]);
  lc.setRow(0, 4, r[4]);
  delay(delaytime);
  lc.setRow(0, 0, d[0]);
  lc.setRow(0, 1, d[1]);
  lc.setRow(0, 2, d[2]);
  lc.setRow(0, 3, d[3]);
  lc.setRow(0, 4, d[4]);
  delay(delaytime);
  lc.setRow(0, 0, u[0]);
  lc.setRow(0, 1, u[1]);
  lc.setRow(0, 2, u[2]);
  lc.setRow(0, 3, u[3]);
  lc.setRow(0, 4, u[4]);
  delay(delaytime);
  lc.setRow(0, 0, i[0]);
  lc.setRow(0, 1, i[1]);
  lc.setRow(0, 2, i[2]);
  lc.setRow(0, 3, i[3]);
  lc.setRow(0, 4, i[4]);
  delay(delaytime);
  lc.setRow(0, 0, n[0]);
  lc.setRow(0, 1, n[1]);
  lc.setRow(0, 2, n[2]);
  lc.setRow(0, 3, n[3]);
  lc.setRow(0, 4, n[4]);
  delay(delaytime);
  lc.setRow(0, 0, o[0]);
  lc.setRow(0, 1, o[1]);
  lc.setRow(0, 2, o[2]);
  lc.setRow(0, 3, o[3]);
  lc.setRow(0, 4, o[4]);
  delay(delaytime);
  lc.setRow(0, 0, 0);
  lc.setRow(0, 1, 0);
  lc.setRow(0, 2, 0);
  lc.setRow(0, 3, 0);
  lc.setRow(0, 4, 0);
  delay(delaytime);
}

void setup()

{
  int runNumber;

  Serial.begin(9600);
  _SERIAL_PRINTLN("");
  _SERIAL_PRINTLN("=========================");
  _SERIAL_PRINTLN("    Dashboard started    ");
  _SERIAL_PRINTLN("=========================");

  runNumber = EEPROM.read(4095) << 8 | EEPROM.read(4096);
  runNumber = runNumber + 1;
  EEPROM.write(4095, (runNumber >> 8));
  EEPROM.write(4096, (runNumber & 0xFF));

  Wire.begin(THIS_NODE_ADDRESS);
  Wire.onReceive(receiveEvent);

  if (SD.begin(SDCardChipSelectPin))
  {
    _SERIAL_PRINT("Using file: ");
    sprintf(lfn, "run%d.txt", runNumber);
    String f = String(lfn);
    lfn[13] = '\0';
    //SdFile::dateTimeCallback(dateTime);
    for (int i = 0; i < 14; i++)
    {
      _SERIAL_PRINT(char(lfn[i]));
    }
    _SERIAL_PRINTLN();
    loggingOnFile = true;
    //                "123456789 123456789 123456789 123456789 123456789 "
    memset(logLine, 32, 80);
    memcpy(&logLine, &"ASC - Dashboard Module started", 30);
    _SERIAL_PRINT("Line to dump:>>");
    logLine[79] = '\0';
    for (int i = 0; i < 80; i++)
    {
      _SERIAL_PRINT(char(logLine[i]));
    }
    _SERIAL_PRINTLN("<<");
    dumpLogLine();
  }
  else
  {
    loggingOnFile = false;
    _SERIAL_PRINTLN("ASC - Communication Module logging failed");
  }
  //
  /*
    The MAX72XX is in power-saving mode on startup,
    we have to do a wakeup call
  */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, 8);
  /* and clear the display */
  lc.clearDisplay(0);
  delay(100);
  writeArduinoOnMatrix(1000);
}

void loop()
{
  //
  // status info received from master
  //
  count++;
  if (logLineReceived)
  {
    dumpLogLine();
    logLineReceived = false;
    Serial.println("Line dumped on file");
  }
  if (matrixBoardDataReceived)
  {
    setMatrixBoard(matrixBoardData);
    matrixBoardDataReceived = false;
  }
}
