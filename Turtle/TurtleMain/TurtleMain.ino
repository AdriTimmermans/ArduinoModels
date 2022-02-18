#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h> // Using I2C communication to communicate between Arduino's on board
#include <LiquidCrystal_I2C.h>
#include<ASCCommunicationDefinition.h>
#include<ASCModelParameters.h>

//#define semafoorPin                 9
#define wakeUpPin                 18

aMessage  mS_sendMessage;
aMessage  mR_receiveMessage;

#define radioActiveLightRed        22
#define radioActiveLightGreen      23
#define justALight                 25

bool          operationsActiveMode = false;
volatile bool checkMessageTimeOut  = false;
int           radioStartTries      = 0;
volatile      operationPossibilities activeOperationalState = standByMode;

operationPossibilities inputOS;
operationPossibilities previousActiveOperationalState = abortedRunMode;

// define vehicle params
String turtleName        = equipmentNameList[5].substring(0, 4);
String groundStationName = equipmentNameList[0].substring(0, 4);
char auxS[10];

long loopCount                 = 99999;
int waitOnRadioCount          = 1;
bool radioIsWorking;
const int i2cLCDTurtleAddress = 0x27;

byte rawContent[maxMessageParameters];

LiquidCrystal_I2C lcdTurtle(i2cLCDTurtleAddress, 16, 2);

// parameters for location determination

byte drivingDirection = drivingForwards;

typedef struct motorMecanumMotionOrderTemplate motorMecanumMotionOrderType;
struct motorMecanumMotionOrderTemplate
{
  byte slaveId;         // I2C id of Arduino Pro Mini used for motion motors (2 motors per MC and H-Bridge)
  byte motorId;         // left side = 1 and 3. Right side = 4 and 6
  byte motorAction;     // 1 = forward, 2 = backward, o = stop
};
motorMecanumMotionOrderType motionMotorOrder;

typedef struct radioMessageGroundStationOrderTemplate GroundStationOrderType;
struct radioMessageGroundStationOrderTemplate
{
  messageCommands topic;
  int             parameter1;
  int             parameter2;
};
volatile  messageTypeIds activeMessage;

typedef struct routeSegmentTemplate routeSegmentType;
struct routeSegmentTemplate
{
  operationPossibilities equipmentOrderMode;
  int equipmentOrderModeParameter1;
  int equipmentOrderModeParameter2;
};

typedef struct equipmentOrderTemplate equipmentOrderType;
struct equipmentOrderTemplate {
  operationPossibilities equipmentOrderMode;
  int equipmentOrderModeParameter1;
  int equipmentOrderModeParameter2;
  gridPositionType positionAtCompletion;
};
equipmentOrderType currentEquipmentOrder;
equipmentOrderType equipmentOrders[10];
byte equipmentOrderPointer = 0;

void displayEquipmentOrders()
{
  _SERIAL_PRINT("In displayEquipmentOrders - available: ");
  _SERIAL_PRINTLN(equipmentOrderPointer);
  for (int i = 0; i < equipmentOrderPointer; i++)
  {
    _SERIAL_PRINT(i);
    _SERIAL_PRINT("\t state ");
    _SERIAL_PRINT(equipmentOrders[i].equipmentOrderMode);
    _SERIAL_PRINT("\t param 1 ");
    _SERIAL_PRINT(equipmentOrders[i].equipmentOrderModeParameter1);
    _SERIAL_PRINT("\t param 2 ");
    _SERIAL_PRINTLN(equipmentOrders[i].equipmentOrderModeParameter2);
  }
}

void addEquipmentOrderSegment (operationPossibilities equipmentOrderModePart, int EOPar1, int EOPar2)
{
  byte LSB, MSB;

  equipmentOrderType oneEquipmentOrder;
  oneEquipmentOrder.equipmentOrderMode = equipmentOrderModePart;
  oneEquipmentOrder.equipmentOrderModeParameter1 = EOPar1;
  oneEquipmentOrder.equipmentOrderModeParameter2 = EOPar2;
  //
  // dummy, only used (for now) in Auto Straddle
  //
  oneEquipmentOrder.positionAtCompletion.x = 0;
  oneEquipmentOrder.positionAtCompletion.y = 0;
  oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid = 0;
  equipmentOrders[equipmentOrderPointer] = oneEquipmentOrder;
  equipmentOrderPointer++;
}

void removeEquipmentOrder()
{
  boolean positionOK = true;
  boolean positionCheckRequired = false;

  if (equipmentOrderPointer >= 1)
  {
    for (int i = 0; i < equipmentOrderPointer; i++)
    {
      equipmentOrders[i] = equipmentOrders[i + 1];
    }
    equipmentOrderPointer--;
    displayEquipmentOrders();
  }
  else
  {
    Serial.println("No orders to remove");
  }
}

equipmentOrderType getNextEquipmentOrder()
{
  return equipmentOrders[0];
}

void removeAllEquipmentOrders()
{
  equipmentOrderPointer = 0;
}

operationPossibilities getNextActiveOperationalState()
{
  operationPossibilities aux = standByMode;

  if (operationsActiveMode)
  {
    if (equipmentOrderPointer > 0)
    {
      _SERIAL_PRINT("Number equip orders: ");
      _SERIAL_PRINTLN(equipmentOrderPointer);
      currentEquipmentOrder = getNextEquipmentOrder();
      _SERIAL_PRINT("Current equipOrderMode = ");
      _SERIAL_PRINT(currentEquipmentOrder.equipmentOrderMode);
      _SERIAL_PRINT(", par 1 = ");
      _SERIAL_PRINT(currentEquipmentOrder.equipmentOrderModeParameter1);
      _SERIAL_PRINT(", par 2 = ");
      _SERIAL_PRINTLN(currentEquipmentOrder.equipmentOrderModeParameter2);
      aux = currentEquipmentOrder.equipmentOrderMode;
    }
    else
    {
      operationsActiveMode = false;
    }
  }
  return aux;
}

bool getRadioStatus()
{
  bool aux = getRadioStatusFromSerial();
  return aux;
}

bool getRadioStatusFromSerial()
{
  bool aux;

  memset (rawContent, 0, sizeof(rawContent));
  rawContent[0] = (byte)checkRadioStatus;

  mS_sendMessage = prepareSerialMessage (turtleName + "M", turtleName + "R", turtleName + "R", radioStatusRequest, 1, &rawContent[0]);
  mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
  displayMessage(&mR_receiveMessage, false);
  aux = ((mR_receiveMessage.messageTypeId == radioStatusReply) && (mR_receiveMessage.content[0] == radioOn));

  return aux;
}
/*
  void checkTimeOuts()
  {

  if (checkMessageTimeOut)
  {
    timeOutCount++;
    _SERIAL_PRINT("waiting...");
    _SERIAL_PRINT(timeOutCount);
    if (timeOutCount > 5)
    {
      messageTimedOut = true;
      timeOutCount = 0;
    }
    _SERIAL_PRINT("time out? ");
    _SERIAL_PRINTLN(messageTimedOut);
  }
  }
*/
void checkForMessage()
{
  //
  if (hasData)
  {
    decodeSerialMessage();
    hasData = false;
  }
}

void stateMessagecombination ( operationPossibilities state, messageCommands messageCode, int messageParameter1, int messageParameter2)
{
  byte auxConvert;
  byte auxMotorId;
  byte auxMotorCommand;

  _SERIAL_PRINT("    in stateMessageCombination with State ");
  _SERIAL_PRINT(state);
  _SERIAL_PRINT(", Messagecode ");
  _SERIAL_PRINT(messageCode);
  _SERIAL_PRINT(" and parameters: ");
  _SERIAL_PRINT(messageParameter1);
  _SERIAL_PRINT(", ");
  _SERIAL_PRINTLN(messageParameter2);

  operationPossibilities operationalStateToOrderList;
  activeOperationalState = abortedRunMode;
  // no state change possible in the following states : stopped, start driving, turn left, turn right, accelerate, decelerate, and locate vehicle
  switch (state)
  {
    case standByMode:
      switch (messageCode) {
        case executeCommands:
          activeOperationalState = executeCommandsMode;
          break;
        case mecanumDrive:
          operationalStateToOrderList = mecanumDriveSetupMode;
          // info from ground control = mecanum drive, mecanum action (1 of 11 possibilities, we now need to make 4 equiporder segments depending on the selected action)
          // coding in mEngineCodes (binary coded engine action byte 8-7 -> engine pos 1, 6-5 -> engine pos 3, 4-3 -> engine pos 4, 1-2 -> engine pos 6
          // messageParameter1 = action:EngineCode translates to 0 = stopped, 1 = forward, 2 = backward for each motor (defined in mEngineCodes in ASCCommunicationDefinition.h)
          auxConvert = mEngineCodes[(byte)messageParameter1 - 1];

          auxMotorId = 1;
          auxMotorCommand = (auxConvert & 0b11000000) >> 6;
          addEquipmentOrderSegment (operationalStateToOrderList, auxMotorId, auxMotorCommand);

          auxMotorId = 3;
          auxMotorCommand = (auxConvert & 0b00110000) >> 4;
          addEquipmentOrderSegment (operationalStateToOrderList, auxMotorId, auxMotorCommand);

          auxMotorId = 4;
          auxMotorCommand = (auxConvert & 0b00001100) >> 2;
          addEquipmentOrderSegment (operationalStateToOrderList, auxMotorId, auxMotorCommand);

          auxMotorId = 6;
          auxMotorCommand = (auxConvert & 0b00000011);
          addEquipmentOrderSegment (operationalStateToOrderList, auxMotorId, auxMotorCommand);

          operationalStateToOrderList = mecanumDriveExecuteMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0);

          displayEquipmentOrders();
          break;
      }
      if (activeOperationalState != executeCommandsMode)
      {
        activeOperationalState = standByMode;
      }
      break;
    case mecanumDriveSetupMode:
      switch (messageCode) {
        case standByMode:
          sendMotorOrdersStop();
          removeAllEquipmentOrders();
          activeOperationalState = standByMode;
          break;
      }
      break;
  }
  _SERIAL_PRINTLN(activeOperationalState);
}

void decodeSerialMessage()
{
  byte aux = processMessageFromSerial();

}

byte processMessageFromSerial()
{
  byte messageTypeId;
  messageCommands returnMessage;
  operationPossibilities currentOS = activeOperationalState;

  GroundStationOrderType aux;
  aux.topic = noContent;
  aux.parameter1 = 0;
  aux.parameter2 = 0;
  routeSegmentType auxSegment;

  memcpy(&mR_receiveMessage, (byte *)byteMessage, sizeof(mR_receiveMessage));
  displayMessage(&mR_receiveMessage, false);
  if (checkCRCByteCorrect(&mR_receiveMessage))
  {
    messageTypeId = mR_receiveMessage.messageTypeId;
    switch (messageTypeId)
    {
      case radioStatusReply:
        radioIsWorking = (mR_receiveMessage.content[0] == radioOn);
        memset (rawContent, 0, sizeof(rawContent));
        rawContent[0] = messageUnderstood;
        mS_sendMessage = prepareSerialMessage (turtleName + "M", turtleName + "R", turtleName + "R", messageStatusReply, 1, &rawContent[0]);
        displayMessage(&mS_sendMessage, true);
        sendStatusMessageSPIFromSlave(wakeUpPin, mS_sendMessage);

        lcdTurtle.clear();
        lcdTurtle.setCursor(0, 0);

        Serial.print("Radio Statusreply received from radio module after heartbeat:");
        if (radioIsWorking)
        {
          Serial.println(" ON");
          lcdTurtle.print("Radio ON");
          digitalWrite(radioActiveLightRed, LOW);
          digitalWrite(radioActiveLightGreen, HIGH);
        }
        else
        {
          Serial.println(" OFF");
          lcdTurtle.print("Radio OFF");
          digitalWrite(radioActiveLightRed, HIGH);
          digitalWrite(radioActiveLightGreen, LOW);
        }
        break;
      case vehicleCommandList:

        lcdTurtle.clear();
        lcdTurtle.setCursor(0, 0);
        _SERIAL_PRINTLN("  vehicleCommandList found ");
        lcdTurtle.print("vehicleCommandList");
        currentOS = activeOperationalState;
        for (int i = 0; i < mR_receiveMessage.totalContentItems; ++i)
        {
          inputOS = activeOperationalState;
          aux.topic = (messageCommands)mR_receiveMessage.content[i * 3];
          aux.parameter1 = mR_receiveMessage.content[i * 3 + 1];
          aux.parameter2 = mR_receiveMessage.content[i * 3 + 2];
          stateMessagecombination (inputOS, aux.topic, aux.parameter1, aux.parameter2);
        }
        // ack to radio module
        memset (rawContent, 0, sizeof(rawContent));
        rawContent[0] = messageUnderstood;
        mS_sendMessage = prepareSerialMessage (turtleName + "M", turtleName + "R", turtleName + "R", messageStatusReply, 1, &rawContent[0]);
        displayMessage(&mS_sendMessage, true);
        sendStatusMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
        break;
      case noMessageActive:

        lcdTurtle.clear();
        lcdTurtle.setCursor(0, 0);
        _SERIAL_PRINTLN("  noMessageActive found ");
        lcdTurtle.print("noMessageActive");
        returnMessage = messageUnderstood;
        break;
      case errorSerialCommunication:

        lcdTurtle.clear();
        lcdTurtle.setCursor(0, 0);
        _SERIAL_PRINTLN("  errorSerialCommunication found ");
        lcdTurtle.print("error SPI Com");
        // nack to gs
        returnMessage = messageUnknown;
        break;
      default:
        lcdTurtle.clear();
        lcdTurtle.setCursor(0, 0);
        _SERIAL_PRINTLN("  -other message- found; id =  ");
        _SERIAL_PRINTLN(messageTypeId);
        lcdTurtle.print("invalid message:");
        lcdTurtle.setCursor(0, 1);
        lcdTurtle.print(messageTypeId);
        returnMessage = messageUnderstood;
        break;
    }
  }
  else
  {
    _SERIAL_PRINTLN("CRC status error");
  }
  //sendMessageAckNackToSerial(returnMessage, messageTypeId);

  // return to current operational state for execution

  return messageTypeId;
}

void executeMotorOrders()
{

  waitFor(100);
  Wire.beginTransmission(0x0B); // transmit to device
  Wire.write(informExecuteMotorOrder);
  Wire.endTransmission();    // stop transmitting
  _SERIAL_PRINTLN ("informExecuteMotorOrder(45) naar I2C-11 (0x0B)");

  Wire.beginTransmission(0x0D); // transmit to device
  Wire.write(informExecuteMotorOrder);
  Wire.endTransmission();    // stop transmitting
  _SERIAL_PRINTLN ("informExecuteMotorOrder(45) naar I2C-13 (0x0D)");
}

void sendMotorOrdersStop ()
{
  _SERIAL_PRINTLN("Stop All Motors");
  transmitI2CMotorMessage(1, 0);
  transmitI2CMotorMessage(3, 0);
  transmitI2CMotorMessage(4, 0);
  transmitI2CMotorMessage(6, 0);
  executeMotorOrders();
  _SERIAL_PRINT("All Motors Stopped");
}

void transmitI2CMotorMessage(byte motorId, byte motorAction)
{
  waitFor(50);
  if ((motorId == 1) || (motorId == 4))
  {
    Wire.beginTransmission(0x0B);
    _SERIAL_PRINT("informMotorOrder(43) naar I2C-11 (0x0B), ");
  }
  else
  {
    Wire.beginTransmission(0x0D);
    _SERIAL_PRINT("informMotorOrder(43) naar I2C-13 (0x0D), ");
  }
  _SERIAL_PRINT(informMotorOrder);
  _SERIAL_PRINT(", ");
  _SERIAL_PRINT(motorId);
  _SERIAL_PRINT(", ");
  _SERIAL_PRINTLN(motorAction);

  Wire.write(informMotorOrder);
  Wire.write(motorId);
  Wire.write(motorAction);

  Wire.endTransmission();    // stop transmitting
}

void setup()
{
  // put your setup code here, to runonce:
  char dummy;

  Serial.begin(74880);
  Wire.begin();
  _SERIAL_PRINTLN("Started");
  lcdTurtle.init();
  lcdTurtle.backlight();
  lcdTurtle.clear();
  lcdTurtle.print("Started");

  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  // turn on interrupts
  //SPCR |= _BV(SPIE);
  SPI.attachInterrupt();

  pinMode(wakeUpPin, OUTPUT);
  digitalWrite(wakeUpPin, HIGH);
  SPIBufferPosition = 0;
  hasData = false;

  pinMode(radioActiveLightRed, OUTPUT);
  digitalWrite(radioActiveLightRed, HIGH);
  pinMode(radioActiveLightGreen, OUTPUT);
  pinMode(justALight, OUTPUT);
  digitalWrite(radioActiveLightGreen, LOW);
  digitalWrite(justALight, LOW);

  lcdTurtle.clear();
  lcdTurtle.print("Wait for radio");
  Serial.println("Start radio polling");
  while (!radioIsWorking)
  {
    if (!startingUp)
    {
      radioStartTries++;
      radioIsWorking = getRadioStatus();
      lcdTurtle.setCursor(0, 1);
      if (radioIsWorking)
      {
        lcdTurtle.print("Radio is operational");
        digitalWrite(radioActiveLightRed, LOW);
        digitalWrite(radioActiveLightGreen, HIGH);
        _SERIAL_PRINTLN("Radio on");
      }
      else
      {
        lcdTurtle.print("Radio off : ");
        lcdTurtle.print(radioStartTries);
        digitalWrite(radioActiveLightRed, HIGH);
        digitalWrite(radioActiveLightGreen, LOW);
        _SERIAL_PRINTLN("Radio off");
        waitFor(5000);
      }
    }
    else
    {
      startingUp = (millis() < 20000);
      waitFor(1000);
    }
  }

  //Timer1.initialize(); // default is 1 second
  //Timer1.attachInterrupt(checkTimeOuts);

  activeOperationalState = standByMode;
  operationsActiveMode = false;

  _SERIAL_PRINTLN("Setup Ready");
  lcdTurtle.setCursor(0, 1);
  lcdTurtle.print("Setup ready");

}

void loop()
{
  char oneChar;
  loopCount++;

  checkForMessage();
  switch (activeOperationalState)
  {
    case executeCommandsMode:
      digitalWrite(justALight, HIGH);
      _SERIAL_PRINT("in executeCommandsMode");
      operationsActiveMode = true;
      previousActiveOperationalState = activeOperationalState;
      activeOperationalState = getNextActiveOperationalState();
      break;
    case standByMode:
      digitalWrite(justALight, LOW);
      if (operationsActiveMode)
      {
        lcdTurtle.clear();
        lcdTurtle.print("Standby mode");
        _SERIAL_PRINTLN("Standby / operationsActive ");
        previousActiveOperationalState = activeOperationalState;
        removeEquipmentOrder();
        activeOperationalState = getNextActiveOperationalState();
      }
      else
      {
        if (activeOperationalState != previousActiveOperationalState)
        {
          removeEquipmentOrder();
        }
        previousActiveOperationalState = activeOperationalState;
        checkForMessage();
      }    //
      break;
    case mecanumDriveSetupMode:
      digitalWrite(justALight, LOW);
      _SERIAL_PRINTLN(" in mecanumDriveSetupMode");
      loopCount = 999;
      if (operationsActiveMode)
      {
        previousActiveOperationalState = activeOperationalState;

        transmitI2CMotorMessage(currentEquipmentOrder.equipmentOrderModeParameter1, currentEquipmentOrder.equipmentOrderModeParameter2);
        removeEquipmentOrder();
      }
      operationsActiveMode = (equipmentOrderPointer > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case mecanumDriveExecuteMode:
      digitalWrite(justALight, HIGH);
      _SERIAL_PRINTLN(" in mecanumDriveExecuteMode");
      loopCount = 999;
      if (operationsActiveMode)
      {
        previousActiveOperationalState = activeOperationalState;
        executeMotorOrders();
        removeEquipmentOrder();
      }
      operationsActiveMode = (equipmentOrderPointer > 0);
      checkForMessage();
      activeOperationalState = getNextActiveOperationalState();
      break;
  }
}
