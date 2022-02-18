//************************************************************
//
// This module handles all Serial communication between the vehicle-
// main-computer and the vehicle-radio-module using serial interface
//
// and
//
// communication with between this vehicle-radio-module and the groundstation-radio-module
// using a mesh WIFI network
//
//************************************************************
#include <EEPROM.h>
#include "namedMesh.h"
#include <SPI.h>
#include<ASCCommunicationDefinition.h>

//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

#define OLED_RESET 0  // GPIO0
#define TX 1
#define RX 3
#define D1 4
#define D2 5
#define D3 0
#define D4 2
#define D0 16
#define D5 14
#define D6 12
#define D7 13
#define D8 15

//Adafruit_SSD1306 display(OLED_RESET);
//uint8_t color = WHITE;
messageTypeIds lastPass = noMessageActive;

int backgroundLightLevelOccurences[10];
String messageAckText;

char line1[25];
char line2[25];

#define radioStatusLightActive   D0
#define radioStatusLightInActive D6
#define activeSearchLight        D8
#define photoCell                A0

String groundStationName = equipmentNameList[0].substring(0, 4);
String pointZeroName     = equipmentNameList[11].substring(0, 4);
String currentRequestor = "";
byte radarMessage[5] = {originNotFound, 0, 0, 0, 0};
int backgroundLightLevel = 0;
bool zeroFree = true;
long lastDisplayAt = 0;
int searchLightLevel = 0;

#define   meshSSId       "AutoStraddleModel"
#define   meshPassword   "TBA01012012"
#define   meshPort       5789

Scheduler     userScheduler; // to control your personal task
namedMesh  mesh;
long lastHeartbeatAt = 0;

Task taskSendMessage( TASK_SECOND * 15, TASK_FOREVER, []()
{
  String msg = String("HB") + pointZeroName;
  String to = groundStationName;
  mesh.sendSingle(to, msg);
  _SERIAL_PRINTLN(msg);
  _SERIAL_PRINTLN();
}); // start with a 15 second interval

volatile bool radioIsActive = false;

aMessage WIFIMessage;

messageTypeIds  WIFIMessageTypeId = noMessageActive;

void nodeTimeAdjustedCallback(int32_t offset)
{
  /*
    _SERIAL_PRINT("ad. ");
    _SERIAL_PRINT(mesh.getNodeTime());
    _SERIAL_PRINT(" - ");
    _SERIAL_PRINTLN(offset);
  */
}

int calculateLightLevel ()
{
  int aux = 0;
  for (int i = 0; i < 7; i++)
  {
    aux += backgroundLightLevelOccurences[i];
  }
  aux = aux / 7;

  return aux;
}

String byteArrayToString (byte source[], byte lengthSource)
{
  String aux = "";
  for (int i = 0; i < lengthSource; i++)
  {
    aux = aux + char(source[i]);
  }
  return aux;
}

void setup()
{
  pinMode(radioStatusLightActive, OUTPUT);
  pinMode(radioStatusLightInActive, OUTPUT);
  pinMode(activeSearchLight, OUTPUT);
  pinMode(photoCell, INPUT);
  digitalWrite(radioStatusLightActive, HIGH);
  digitalWrite(radioStatusLightInActive, HIGH);
  digitalWrite(activeSearchLight, HIGH);

  waitFor(2500);
  Serial.begin(74880);
  EEPROM.begin(100);
  thisCHEID = EEPROM.read(0);
  _SERIAL_PRINTLN(thisCHEID);
  thisCHEString = equipmentNameList[thisCHEID].substring(0, 4);
  _SERIAL_PRINTLN("==========================");
  _SERIAL_PRINTLN("    ");
  _SERIAL_PRINT(thisCHEString);
  _SERIAL_PRINTLN(" RADIO MODULE    ");
  _SERIAL_PRINTLN("==========================");

  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //display.display();

  for (int i = 0; i < 7; i++)
  {
    backgroundLightLevelOccurences[i] = analogRead(photoCell);
    digitalWrite(activeSearchLight, LOW);
    waitFor(100);
    digitalWrite(activeSearchLight, HIGH);
  }

  digitalWrite(radioStatusLightActive, LOW);
  digitalWrite(radioStatusLightInActive, HIGH);
  digitalWrite(activeSearchLight, LOW);
  backgroundLightLevel = calculateLightLevel();
  zeroFree = true;

  //display.clearDisplay();
  //display.setTextSize(2);
  //display.setTextColor(WHITE);
  //display.setCursor(0, 0);
  //display.println("SLEEP");
  //display.display();
  //waitFor(1);

#ifdef DEBUG_PRINT
  //mesh.setDebugMsgTypes(ERROR | DEBUG | CONNECTION | STARTUP);  // set before init() so that you can see startup messages
  mesh.setDebugMsgTypes(ERROR);  // set before init() so that you can see startup messages
#endif

  mesh.init(meshSSId, meshPassword, &userScheduler, meshPort);
  mesh.setName(thisCHEString); // This needs to be an unique name!
  mesh.onReceive([] (uint32_t from, String & msg )
  {
    _SERIAL_PRINT(thisCHEString);
    _SERIAL_PRINT(" received following message: ");
    _SERIAL_PRINTLN(msg);
    if (strncmp(&msg[0], "HB", 2) == 0)
    {
      _SERIAL_PRINTLN("'heartbeat' found");
      radioIsActive = true;
      lastHeartbeatAt = millis();
      digitalWrite(radioStatusLightActive, HIGH);
      digitalWrite(radioStatusLightInActive, LOW);
    }
    else
    {
      _SERIAL_PRINT(" Radio message found :");
      WIFIMessage = receiveWIFIUnsolicitedData(&msg);
      WIFIMessageTypeId = (messageTypeIds)WIFIMessage.messageTypeId;
    }
  });
  mesh.onChangedConnections([]()
  {
    //_SERIAL_PRINTLN("Connection change");
  });
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
  WIFIMessageTypeId = noMessageActive;
  lastHeartbeatAt = -61000;
}

void drawLightLevel (int sensorValue, int pixelY, bool clearDisplayFirst)
{
  if (clearDisplayFirst)
  {
    //display.clearDisplay();
  }
  //display.setTextSize(2);
  //display.setTextColor(WHITE);
  //display.setCursor(3, pixelY);
  //display.println(sensorValue);
  //display.display();
  //waitFor(1);
}

void sendWIFIMessage (String __messageTarget, aMessage __WIFIMessage)
{

  String auxValue;
  String auxString;
  int indexContent = 0;


  auxString = "";

  for (int i = 0; i < 3; i++)
  {
    auxValue = String("000") + String(__WIFIMessage.startOfMessage[i]);
    auxString += auxValue.substring(auxValue.length() - 3);
  }

  auxValue = String("000") + String(__WIFIMessage.totalMessageLength);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.totalContentItems);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.messageNumberHighByte);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.messageNumberLowByte);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.CRCByte);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.mediumId);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.senderId);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.addresseeId);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.finalAddressId);
  auxString += auxValue.substring(auxValue.length() - 3);
  auxValue = String("000") + String(__WIFIMessage.messageTypeId);
  auxString += auxValue.substring(auxValue.length() - 3);
  for (int i = 0; i < __WIFIMessage.totalContentItems + 1; i++)
  {
    auxValue = String("000") + String(__WIFIMessage.content[indexContent]);
    auxString += auxValue.substring(auxValue.length() - 3);
    auxValue = String("000") + String(__WIFIMessage.content[indexContent + 1]);
    auxString += auxValue.substring(auxValue.length() - 3);
    auxValue = String("000") + String(__WIFIMessage.content[indexContent + 2]);
    auxString += auxValue.substring(auxValue.length() - 3);
    indexContent = indexContent + 3;
  }

  auxString += "\n";

  String to = __messageTarget;
  mesh.sendSingle(to, auxString);
  _SERIAL_PRINTLN();
  _SERIAL_PRINT("Radio message sent: ");
  _SERIAL_PRINTLN(auxString);
  _SERIAL_PRINTLN();

  auxString += "\n";
}

void loop()
{
  mesh.update();

  if (radioIsActive)
  {
    if ((millis() - lastHeartbeatAt) > 60000)
    {
      radioIsActive = false;
      digitalWrite(radioStatusLightActive, LOW);
      digitalWrite(radioStatusLightInActive, HIGH);
      Serial.println("Heartbeat dropped after one minute");
    }
  }

  if (radioIsActive)
  {
    //
    // radio message
    // xxx = message Id
    //

    if  (WIFIMessageTypeId != noMessageActive)
    {
      _SERIAL_PRINT("WIFIMessageTypeId = ");
      _SERIAL_PRINTLN(WIFIMessageTypeId);
      currentRequestor = equipmentNameList[WIFIMessage.senderId].substring(0, 4);
      if (WIFIMessageTypeId != messageStatusReply)
      {
        messageNumber++;
        if (zeroFree)
        {
          WIFIMessage.content[0] = messageUnderstood;
          WIFIMessage.content[1] = 0;
          WIFIMessage.content[2] = 0;

          WIFIMessage = prepareWIFIMessage (thisCHEString + "R", currentRequestor + "R", currentRequestor + "R", messageStatusReply, 1, WIFIMessage.content);
          sendWIFIMessage (currentRequestor, WIFIMessage) ; // the ack
          displayMessage(&WIFIMessage, true);
          _SERIAL_PRINTLN("WIFI message Understood sent to requestor");
          zeroFree = false;
          digitalWrite(activeSearchLight, HIGH);
          displayMessage(&WIFIMessage, true);
          drawLightLevel (searchLightLevel, 2, true);
          drawLightLevel (backgroundLightLevel, 24, false);
          lastDisplayAt = millis();
        }
        else
        {
          WIFIMessage.content[0] = messageUnknown;
          WIFIMessage.content[1] = 0;
          WIFIMessage = prepareWIFIMessage (thisCHEString + "R", currentRequestor + "R", currentRequestor + "R", messageStatusReply, 1, WIFIMessage.content);
          sendWIFIMessage (currentRequestor, WIFIMessage) ; // the ack
          _SERIAL_PRINTLN("Point zero busy");
          displayMessage(&WIFIMessage, true);

        }
      }
      WIFIMessageTypeId = noMessageActive;
    }
    for (int i = 0; i < 6; i++)
    {
      backgroundLightLevelOccurences[i] = backgroundLightLevelOccurences[i + 1];
    }
    backgroundLightLevelOccurences[6] = analogRead(photoCell);
    waitFor(50);

    if (!zeroFree)
    {
      searchLightLevel = calculateLightLevel();
      drawLightLevel (searchLightLevel, 2, true);
      drawLightLevel (backgroundLightLevel, 24, false);
      if (searchLightLevel > (backgroundLightLevel + 20))
      {
        _SERIAL_PRINTLN("Origin found");
        radarMessage[0] = originFound;
        WIFIMessage = prepareWIFIMessage (pointZeroName + "R", currentRequestor + "R", currentRequestor + "M", searchOriginReply, 1, radarMessage);
        _SERIAL_PRINTLN("Point zero busy");
        zeroFree = true;
        //display.clearDisplay();
        //display.setTextSize(2);
        //display.setTextColor(WHITE);
        //display.setCursor(0, 0);
        //display.println("SLEEP");
        //display.display();
        //waitFor(1);
        drawLightLevel (backgroundLightLevel, 24, false);

        digitalWrite(activeSearchLight, LOW);
        sendWIFIMessage (currentRequestor, WIFIMessage) ;
      }
    }
    else
    {
      for (int i = 0; i < 6; i++)
      {
        backgroundLightLevelOccurences[i] = backgroundLightLevelOccurences[i + 1];
      }
      backgroundLightLevelOccurences[6] = analogRead(photoCell);
      backgroundLightLevel = calculateLightLevel();
      if ((millis() - lastDisplayAt) > 5000)
      {
        //display.clearDisplay();
        //display.setTextSize(2);
        //display.setTextColor(WHITE);
        //display.setCursor(0, 0);
        //display.println("SLEEP");
        //display.display();
        //waitFor(1);
        drawLightLevel (backgroundLightLevel, 24, false);
        lastDisplayAt = millis();
      }
    }
  }
  else
  {
    digitalWrite(radioStatusLightActive,   LOW);
    digitalWrite(radioStatusLightInActive, HIGH);
  }

}
