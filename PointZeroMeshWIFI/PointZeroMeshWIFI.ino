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

messageTypeIds lastPass = noMessageActive;

String messageAckText;

char line1[25];
char line2[25];

#define radioStatusLightActive    D0
#define radioStatusLightInActive  D6
#define activeSearchLight         D8
#define pinInMux                  A0
#define pinOutS0                  D5
#define pinOutS1                  D7

int muxState[4] = {0};

String groundStationName = equipmentNameList[0].substring(0, 4);
String pointZeroName     = equipmentNameList[11].substring(0, 4);
String currentRequestor = "";
byte radarMessage[5] = {originNotFound, 0, 0, 0, 0};
float lightLevel[4] = {0};
bool zeroFree = true;
float referenceLightLevel[4] = {0};

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

void updateMux()
{
  digitalWrite(pinOutS0, HIGH);
  digitalWrite(pinOutS1, LOW);
  waitFor(10);
  muxState[0] = analogRead(pinInMux);
  digitalWrite(pinOutS0, LOW);
  digitalWrite(pinOutS1, HIGH);
  waitFor(10);
  muxState[1] = analogRead(pinInMux);
}

/*
  String byteArrayToString (byte source[], byte lengthSource)
  {
  String aux = "";
  for (int i = 0; i < lengthSource; i++)
  {
    aux = aux + char(source[i]);
  }
  return aux;
  }
*/
void setStartLightLevels()
{
  bool lightOn = true;
  for (int j = 0; j < 7; j++)
  {
    updateMux();
    for (int i = 0; i < 2; i++)
    {
      lightLevel[i] = 0.8 * lightLevel[i] + 0.2 * (float)muxState[i];
    }
    (lightOn) ?    digitalWrite(activeSearchLight, HIGH) :    digitalWrite(activeSearchLight, LOW);
    lightOn = !lightOn;
  }
  lightOn = false;
}

void setup()
{
  pinMode(radioStatusLightActive, OUTPUT);
  pinMode(radioStatusLightInActive, OUTPUT);
  pinMode(activeSearchLight, OUTPUT);
  pinMode(pinOutS0, OUTPUT);
  pinMode(pinOutS1, OUTPUT);
  pinMode(pinInMux, INPUT);

  digitalWrite(radioStatusLightActive, HIGH);
  digitalWrite(radioStatusLightInActive, HIGH);
  digitalWrite(activeSearchLight, HIGH);
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

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //display.display();
  digitalWrite(radioStatusLightActive, LOW);
  digitalWrite(radioStatusLightInActive, HIGH);
  digitalWrite(activeSearchLight, LOW);
  zeroFree = true;

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
          radarMessage[0] = messageUnderstood;
          radarMessage[1] = 0;
          radarMessage[2] = 0;

          WIFIMessage = prepareWIFIMessage (thisCHEString + "R", currentRequestor + "R", currentRequestor + "R", messageStatusReply, 1, &radarMessage[0]);
          sendWIFIMessage (currentRequestor, WIFIMessage) ; // the ack
          displayMessage(&WIFIMessage, true);
          _SERIAL_PRINTLN("WIFI message Understood sent to requestor");
          zeroFree = false;
          setStartLightLevels();
          digitalWrite(activeSearchLight, HIGH);
          displayMessage(&WIFIMessage, true);
        }
        else
        {
          radarMessage[0] = messageUnknown;
          radarMessage[1] = 0;
          radarMessage[2] = 0;
          
          WIFIMessage = prepareWIFIMessage (thisCHEString + "R", currentRequestor + "R", currentRequestor + "R", messageStatusReply, 1, &radarMessage[0]);
          sendWIFIMessage (currentRequestor, WIFIMessage) ; // the ack
          _SERIAL_PRINTLN("Point zero busy");
          displayMessage(&WIFIMessage, true);

        }
      }
      WIFIMessageTypeId = noMessageActive;
    }
    //
    // if searchlight activated (zeroFree = false) then check change in at least one of the 4 sensor levels
    //
    if (!zeroFree)
    {
      //
      // save current base level:
      //
      for (int i = 0; i < 2; i++)
      {
        referenceLightLevel[i] = lightLevel[i];
      }
      //
      // read new light levels
      //
      updateMux(); // using self designed multiplexer
      for (int i = 0; i < 2; i++)
      {
        if (i == 1)
        {
          Serial.println(muxState[i]);
        }
        else
        {
          Serial.print(muxState[i]);
          Serial.print(",");
        }
      }
      //
      // update light level with high noise filter
      //
      for (int i = 0; i < 2; i++)
      {
        lightLevel[i] = 0.8 * lightLevel[i] + 0.2 * (float)muxState[i];

        if ((lightLevel[i] > referenceLightLevel[i] + 20.0))
        {
          zeroFree = zeroFree | true;
        }
      }
      //
      // if at least one of the sensors has reached the level, send an radiomessage that the searchlight has been found and reset the zeroFree value to true
      //
      if (zeroFree)
      {
        _SERIAL_PRINTLN("Origin found");
        radarMessage[0] = originFound;
        radarMessage[1] = 0;
        radarMessage[2] = 0;
        WIFIMessage = prepareWIFIMessage (pointZeroName + "R", currentRequestor + "R", currentRequestor + "M", searchOriginReply, 1, &radarMessage[0]);
        displayMessage(&WIFIMessage, true);
        zeroFree = true;
        digitalWrite(activeSearchLight, LOW);
        sendWIFIMessage (currentRequestor, WIFIMessage) ;
      }
    }
  }
  else
  {
    digitalWrite(radioStatusLightActive,   LOW);
    digitalWrite(radioStatusLightInActive, HIGH);
  }
}
