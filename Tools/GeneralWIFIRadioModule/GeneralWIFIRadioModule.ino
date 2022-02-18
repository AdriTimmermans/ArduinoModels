//************************************************************
//
// This module handles all Serial communication between the
// main-computer and the radio-module of any equipment using serial interface
//
// and
//
// communication over the WIFI network
//
// This module and pinout is based on the NodeMCU ESP8266 V1.0
//
//************************************************************
#include <EEPROM.h>
#include "namedMesh.h"
#include<SPI.h>
#include<Wire.h>
#include <LiquidCrystal_I2C.h>
#include<ASCCommunicationDefinition.h>

#if defined(ESP8266)
int SPIChipSelectPin = D8;
int receiveSlaveInterruptPin = D4;
#else
int SPIChipSelectPin          = SS;
int receiveSlaveInterruptPin  = A4;  // = pin 32
#endif

volatile bool networkActive = false;
volatile long networkActiveSince = 0;

String to;
String msg;

#define         meshSSId       "AutoStraddleModel"
#define         meshPassword   "TBA01012012"
#define         meshPort       5789
int             heartbeatCountOut = 0;
int             heartbeatCountIn = 0;
const int i2cRadioAddress = 0x27;
LiquidCrystal_I2C lcdRadio(i2cRadioAddress, 20, 4);

Scheduler     userScheduler; // to control your personal task
namedMesh  mesh;

// eeprom 0 = this equipment ID (See ASCCommunicationDefinition.h
// eeprom 1 - 50 = ID of equipment that will receive a heartbeat signal


Task taskSendMessage( TASK_SECOND * 15, TASK_FOREVER, []()
{
  heartbeatCountOut++;
  lcdRadio.setCursor(0, 1);
  lcdRadio.print("                    ");
  lcdRadio.setCursor(0, 2);
  lcdRadio.print("                    ");
  lcdRadio.setCursor(0, 1);
  lcdRadio.print("Heartbeat out");
  _SERIAL_PRINTLN ("Sending heartbeats");
  lcdRadio.print (" ");
  lcdRadio.print(heartbeatCountOut);
  for (int i = 0; i < (maxEquipmentNumber - 3) / 2; i++)
  {
    if (EEPROM.read(i + 1) == 1)
    {
      to = equipmentNameList[i].substring(0, 4);
      msg = String("HB") + thisCHEString;
      mesh.sendSingle(to, msg);
      _SERIAL_PRINT(msg);
      _SERIAL_PRINTLN("has been sent");
    }
  }
}); // start with a 15 second interval

byte radioStatusMessage[4] = {radioOff, 0, 0, 0};

aMessage WIFIMessage;

aMessage mS_sendMessage;
aMessage mR_receiveMessage;

byte rawContent[maxSerialBufferLength];

volatile messageTypeIds  WIFIMessageTypeId = noMessageActive;

void nodeTimeAdjustedCallback(int32_t offset)
{
  /*
    _SERIAL_PRINT("ad. ");
    _SERIAL_PRINT(mesh.getNodeTime());
    _SERIAL_PRINT(" - ");
    _SERIAL_PRINTLN(offset);
  */
}

void setup()
{
  waitFor(2500);
  Serial.begin(74880);
  EEPROM.begin(100);
  Wire.begin();
  thisCHEID = EEPROM.read(0);
  _SERIAL_PRINTLN(thisCHEID);
  thisCHEString = equipmentNameList[thisCHEID].substring(0, 4);
  _SERIAL_PRINTLN("==========================");
  _SERIAL_PRINTLN("    ");
  _SERIAL_PRINT(thisCHEString);
  _SERIAL_PRINTLN(" RADIO MODULE    ");
  _SERIAL_PRINTLN("==========================");
  lcdRadio.init();
  lcdRadio.backlight();
  lcdRadio.clear();
  lcdRadio.print(thisCHEString);
  lcdRadio.print(" RM started");

  SPI.begin();
  pinMode(receiveSlaveInterruptPin, INPUT_PULLUP);
  pinMode(SPIChipSelectPin, OUTPUT);
  digitalWrite(SPIChipSelectPin, LOW);
  attachInterrupt(digitalPinToInterrupt(receiveSlaveInterruptPin), setDataOnSPIFlag, FALLING);

  dataOnSPI = false;

#ifdef DEBUG_PRINT
  mesh.setDebugMsgTypes(ERROR);  // set before init() so that you can see startup messages
  //mesh.setDebugMsgTypes(ERROR | DEBUG | CONNECTION | STARTUP);  // set before init() so that you can see startup messages
#endif

  mesh.init(meshSSId, meshPassword, &userScheduler, meshPort);
  mesh.setName(thisCHEString); // This needs to be an unique name!
  mesh.onReceive([] (uint32_t from, String & msg )
  {
    _SERIAL_PRINT(thisCHEString);
    _SERIAL_PRINT(" received following message: ");
    _SERIAL_PRINTLN(msg);
    if (!networkActive)
    {
      networkActive = true;
      networkActiveSince = 0;
    }


    if (strncmp(&msg[0], "HB", 2) == 0)
    {
      _SERIAL_PRINTLN("'HB' found");
      lcdRadio.setCursor(0, 3);
      lcdRadio.print("                    ");
      lcdRadio.setCursor(0, 4);
      lcdRadio.print("                    ");
      lcdRadio.setCursor(0, 3);
      lcdRadio.print("Heartbeat in ");
      heartbeatCountIn++;
      lcdRadio.print(heartbeatCountIn);
    }
    else
    {
      _SERIAL_PRINT(" Radio message found :");
      Serial.println(msg);
      WIFIMessage = receiveWIFIUnsolicitedData(&msg);
      WIFIMessageTypeId = (messageTypeIds)WIFIMessage.messageTypeId;
      lcdRadio.setCursor(0, 3);
      lcdRadio.print("                    ");
      lcdRadio.setCursor(0, 4);
      lcdRadio.print("                    ");
      lcdRadio.setCursor(0, 3);
      lcdRadio.print(msg.substring(0, 19));
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
  _SERIAL_PRINT("Setup done");
}

void sendWIFIMessage (String __messageTarget, aMessage __WIFIMessage)
{

  String auxValue;
  String auxString;
  int indexContent = 0;

  if (networkActive)
  {
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
    lcdRadio.setCursor(0, 2);
    lcdRadio.print("Sending message :");
    lcdRadio.setCursor(0, 3);
    lcdRadio.print(auxString.substring(0, 19));

    String to = __messageTarget;
    mesh.sendSingle(to, auxString);
    _SERIAL_PRINTLN();
    _SERIAL_PRINT("Radio message sent: ");
    _SERIAL_PRINTLN(auxString);
    _SERIAL_PRINTLN();
  }
  else
  {
    _SERIAL_PRINTLN();
    _SERIAL_PRINTLN("Network not active so no message sent. ");
    _SERIAL_PRINTLN();
  }
}

void handleSerialMessageOnRadioModuleGeneral(int SPIChipSelectPin, aMessage mA_Message)
{
  String msgSender = toDef(mA_Message.senderId).substring(0, 4);
  String msgAddressee = toDef(mA_Message.finalAddressId).substring(0, 4);
  lcdRadio.setCursor(0, 1);
  switch ((messageTypeIds)mA_Message.messageTypeId)
  {
    case noMessageActive:
      lcdRadio.print("NoMsgActive");
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;      
    case radioStatusRequest:
      lcdRadio.print("RadioStatReq");
      _SERIAL_PRINTLN("Send reply on radio status request");
      mS_sendMessage = prepareSerialMessage (msgSender + "R", msgSender + "M", msgSender + "M", radioStatusReply, 1,  &rawContent[0]);
      displayMessage(&mS_sendMessage, true);
      mR_receiveMessage = sendMessageSPIFromMaster(SPIChipSelectPin, mS_sendMessage);
      displayMessage(&mR_receiveMessage, true);
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
    case vehicleStateList:
      lcdRadio.print("vehStateList");
      WIFIMessage = prepareWIFIMessage (msgSender + "R", msgAddressee + "R", msgAddressee + "M", vehicleStateList, mA_Message.totalContentItems,  &mA_Message.content[0]);
      sendWIFIMessage (msgAddressee, WIFIMessage) ; // the list of vehicle commands (minimum = 1 command)
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
    case vehicleCommandList:
      lcdRadio.print("vehComList");
      WIFIMessage = prepareWIFIMessage (msgSender + "R", msgAddressee + "R", msgAddressee + "M", vehicleCommandList, mA_Message.totalContentItems, &mA_Message.content[0]);
      sendWIFIMessage (msgAddressee, WIFIMessage) ; // the list of vehicle commands (min = 1 command)
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
    case locationRequest:
      lcdRadio.print("locReq");
      WIFIMessage = prepareWIFIMessage (msgSender + "R", msgAddressee + "R", msgAddressee + "M", locationRequest, mA_Message.totalContentItems,  &mA_Message.content[0]);
      sendWIFIMessage (msgAddressee, WIFIMessage) ;
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
    case searchOriginRequest:
      lcdRadio.print("locReq");
      WIFIMessage = prepareWIFIMessage (msgSender + "R", msgAddressee + "R", msgAddressee + "R", searchOriginRequest, mA_Message.totalContentItems,  &mA_Message.content[0]);
      sendWIFIMessage (msgAddressee, WIFIMessage) ;
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
    case locationReply:
      lcdRadio.print("locReply");
      WIFIMessage = prepareWIFIMessage (msgSender + "R", msgAddressee + "R", msgAddressee + "M", locationReply, mA_Message.totalContentItems,  &mA_Message.content[0]);
      sendWIFIMessage (msgAddressee, WIFIMessage) ;
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;

    case messageStatusReply:
      lcdRadio.print("msgStatReply");
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
    default:
      lcdRadio.print("Invalid req:");
      lcdRadio.setCursor(13, 1);
      lcdRadio.print(mR_receiveMessage.messageTypeId);
      _SERIAL_PRINT("No valid function for this Mesh: ");
      _SERIAL_PRINTLN(mR_receiveMessage.messageTypeId);
      mR_receiveMessage.messageTypeId = 0; // noMessageActive
      break;
  }

}

void handleWIFIMessageOnRadioModuleGeneral(int SPIChipSelectPin)
{

  String msgSender = toDef(WIFIMessage.senderId).substring(0, 4);
  if ((messageTypeIds)WIFIMessage.messageTypeId != messageStatusReply)
  {
    //
    // create Serial message before overwriting the WIFI message data to perform an ack
    //
    mS_sendMessage = prepareSerialMessage (thisCHEString + "R", thisCHEString + "M", thisCHEString + "M", (messageTypeIds)WIFIMessage.messageTypeId, WIFIMessage.totalContentItems,  &WIFIMessage.content[0]);
    _SERIAL_PRINTLN("Message received over WIFI passed on to main module");
    displayMessage(&mS_sendMessage, true);
    mR_receiveMessage = sendMessageSPIFromMaster(SPIChipSelectPin, mS_sendMessage);
    //
    // handle message from network
    //

    //
    // create WIFI ack message
    //
    WIFIMessage.content[0] = messageUnderstood;
    WIFIMessage.content[1] = 0;
    WIFIMessage.content[2] = 0;
    WIFIMessage = prepareWIFIMessage (thisCHEString + "R", msgSender + "R", msgSender + "R", messageStatusReply, 1,  WIFIMessage.content);
    displayMessage(&WIFIMessage, true);
    //
    // send message via radio
    //
    sendWIFIMessage (msgSender, WIFIMessage) ; // the ack
    _SERIAL_PRINTLN("WIFI messageStatusReply sent");
  }
}
void loop() {

  mesh.update();
  //
  // handle message from equipment computer
  //
  if (dataOnSPI)
  {
    lcdRadio.clear();
    lcdRadio.setCursor(0, 0);
    lcdRadio.print("Ping received");
    mR_receiveMessage = receiveSPIMessageOnMaster(SPIChipSelectPin);
    displayMessage(&mR_receiveMessage, false);
    dataOnSPI = false;
    handleSerialMessageOnRadioModuleGeneral(SPIChipSelectPin, mR_receiveMessage);
  }
  //
  // check WIFI network and, if applicable, handle message from WIFI network
  //

  if (networkActive)
  {
    if (networkActiveSince == 0)
    {
      networkActiveSince = millis();
      if (!((thisCHEID == 11) || (thisCHEID == 12))) // Zero does not have a main module
      {
        memset(rawContent, 0, sizeof(rawContent));
        rawContent[0] = radioOn;
        mS_sendMessage = prepareSerialMessage (equipmentNameList[thisCHEID + 13], equipmentNameList[thisCHEID], equipmentNameList[thisCHEID], radioStatusReply, 1,  &rawContent[0]);
        mR_receiveMessage = sendMessageSPIFromMaster(SPIChipSelectPin, mS_sendMessage);
      }
    }
    else
    {
      //
      // handle message from main computer
      //
      if (WIFIMessageTypeId != noMessageActive)
      {
        handleWIFIMessageOnRadioModuleGeneral(SPIChipSelectPin);
        WIFIMessageTypeId = noMessageActive;
      }
    }
  }
}
