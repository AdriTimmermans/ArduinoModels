#include <EEPROM.h>
#include <LinkedList.h>
#include <SPI.h>
#include<ASCCommunicationDefinition.h>
//#define TEST_MODE 1

#define wakeUpPin 18

int numberOfReceivedMessages;

String        equipmentName;
byte          equipmentId;
String        groundStationName       = equipmentNameList[0].substring(0, 4); // Name needs to be unique
bool          radioIsWorking          = false;
int           radioStartTries         = 0;
bool          previousStatus          = true;
String        userInput;
byte          groundStationId         = 0;
int           turnRadius              = 48;
char          lcdLine1[25];
char          lcdLine2[25];
long          loopcount               = 0;
volatile long biasTime;
byte          pointerInBuffer         = 0;
byte          characterBuffer[255];
byte          numberOfCommands        = 0;
int           messagesFromInput       = 0;
bool          positionRequestPending  = false;
int           loopCount               = 0;
int           radioStatusLightPin     = 22;
byte          rawContent[maxMessageParameters];
int           lineNr                  = 0;
int           posNr                   = 0;
bool          newOrderFound           = false;
long          testWaitTimeStarted     = 0;

typedef struct blockedAreaType oneBlockedArea;
LinkedList<oneBlockedArea> blockedAreas = LinkedList<oneBlockedArea>();
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
char oneChar;

struct blockedAreaType {
  int x1;
  int y1;
  int x2;
  int y2;
};

oneBlockedArea gridOrigin;

typedef struct routePointType oneRoutePoint;
struct routePointType {
  int x;
  int y;
  int orientation;
  operationPossibilities drivingDirection;
};
typedef struct orderListType oneOrder;
LinkedList<oneOrder> orderList = LinkedList<oneOrder>();

aMessage mR_receiveMessage;
aMessage mS_sendMessage;
bool sentSuccess;

struct orderListType {
  messageCommands operation;
  /*  struct {
      int x;
      int y;
      int orientation;
      operationPossibilities drivingDirection;
    }*/
  oneRoutePoint routePoint;
};

oneRoutePoint currentGridPosition;
oneRoutePoint virtualGridPosition;
oneRoutePoint pickUpGridPosition;
oneRoutePoint routeEndGridPosition;
oneRoutePoint enteredRouteEndGridPosition;
oneRoutePoint enteredPickUpGridPosition;

bool displayMessages = true;
bool lastMessageOK = true;
messageTypeIds  groundStationCommunicationModuleMessageId;

long inputMessageIndicator;

byte nrChars;
boolean fullMessageReceived = false;
boolean messageReceived = false;

bool searchVehicle = true;
bool continueDecouserInput = true;

String orders;
String inputLine;
String auxValue;
String auxString = "";

bool getRadioStatus()
{
  bool aux = getRadioStatusFromSerial();
  return aux;
}

bool getRadioStatusFromSerial()
{
  int bytesSend;
  bool aux;

  memset (rawContent, 0, sizeof(rawContent));
  rawContent[0] = (byte)checkRadioStatus;

  mS_sendMessage = prepareSerialMessage (groundStationName + "M", groundStationName + "R", groundStationName + "R", radioStatusRequest, 1, &rawContent[0]);
  displayMessage(&mS_sendMessage, true);
  mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
  displayMessageLCD(&mR_receiveMessage);
  displayMessage(&mR_receiveMessage, false);
  aux = (mR_receiveMessage.messageTypeId == radioStatusReply) && (mR_receiveMessage.content[0] == radioOn);

  return aux;
}

void displayMessageLCD (aMessage * mSg)
{

  // preparations

  lineNr = 0;
  posNr = 0;
  lcd.clear();

  // line 0:

  lcd.setCursor(0, 0);
  lcd.print(mSg->totalMessageLength);    //  Length of the message
  lcd.setCursor(4, 0);
  lcd.print(mSg->totalContentItems);     //  Number of Items to be added in the content of the message (3 bytes per item)
  lcd.setCursor(8, 0);
  lcd.print(((mSg->messageNumberHighByte << 8) | mSg->messageNumberLowByte));         //  two byte integer to identify the message, in combination with the senderCode
  lcd.setCursor(17, 0);
  lcd.print(mSg->CRCByte);               //  Control byte. Least significant byte of sum of all bytes

  // line 1

  lcd.setCursor(0, 1);
  lcd.print(toDef(mSg->mediumId));
  lcd.print(toDef(mSg->senderId));
  lcd.print(toDef(mSg->addresseeId));
  lcd.print(toDef(mSg->finalAddressId));

  // line 2 / 3

  lcd.setCursor(0, 2);
  lcd.print(mSg->messageTypeId);
  lineNr = 2;
  posNr = 0;
  for (int i = 0; i < ((mSg->totalContentItems + 1) * 3); i++)
  {
    posNr += 4;
    if (posNr > 19)
    {
      posNr = 0;
      lineNr++;
    }
    if (lineNr  < 4)
    {
      lcd.setCursor(posNr, lineNr);
      lcd.print(mSg->content[i]);
    }
  }
}

void requestForLocation(byte eCode)
{

  memset (rawContent, 0, sizeof(rawContent));
  rawContent[0] = requestPosition;
  rawContent[3] = executeCommands;

  loopCount = 0;

  equipmentName = equipmentNameList[eCode];
  mS_sendMessage = prepareSerialMessage (groundStationName + "M", groundStationName + "R", equipmentName + "M", locationRequest, 2, &rawContent[0]);
  mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);

  decodeCurrentPositionReply();
  currentGridPosition.drivingDirection = forwardDrivingMode;
  auxString = "position:";
  auxValue = String("    ") + String((currentGridPosition.x) / 10);
  auxString += auxValue.substring(auxValue.length() - 4);
  auxValue = String("    ") + String((currentGridPosition.y) / 10);
  auxString += auxValue.substring(auxValue.length() - 4);
  auxValue = String("    ") + String(currentGridPosition.orientation);
  auxString += auxValue.substring(auxValue.length() - 4);

  Serial.println(auxString);
}

void decodeCurrentPositionReply()
{
  // routepoint definition in A-SC program is of a different type, so we need an auxiliary variable to read it

  _SERIAL_PRINT("Position received: (");

  currentGridPosition.x                = ((mR_receiveMessage.content[0]) << 8) | mR_receiveMessage.content[1];
  currentGridPosition.y                = ((mR_receiveMessage.content[3]) << 8) | mR_receiveMessage.content[4];
  currentGridPosition.orientation      = ((mR_receiveMessage.content[6]) << 8) | mR_receiveMessage.content[7];
  currentGridPosition.drivingDirection = mR_receiveMessage.content[9];

  _SERIAL_PRINT(currentGridPosition.x);
  _SERIAL_PRINT(" ");
  _SERIAL_PRINT(currentGridPosition.y);
  _SERIAL_PRINT(" ");
  _SERIAL_PRINT(currentGridPosition.orientation);
  _SERIAL_PRINTLN();
}

bool flushUntilStartOfMessage()
{
  bool aux = false; // no start of message found, only noise
  const char A = 'A';
  const char B = 'B';
  const char Z = 'Z';
  char one = ' ';
  char two = ' ';
  char three;
  bool continueSearching = true;
  lineNr = 0;
  posNr = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  do
  {
    posNr++;
    if (posNr == 20)
    {
      posNr = 0;
      lineNr++;
      if (lineNr == 5)
      {
        posNr = 19;
      }
    }
    three = Serial.read();
    lcd.setCursor(posNr, lineNr);
    lcd.print(three);
    continueSearching = (!((one == A) && (two == B) && (three == Z))) && (Serial.available() > 0);
    if (continueSearching)
    {
      one = two;
      two = three;
    }
  }
  while (continueSearching);
  aux = ((one == A) && (two == B) && (three == Z));
  if (aux)
  {
    characterBuffer[0] = A;
    characterBuffer[1] = B;
    characterBuffer[2] = Z;
    pointerInBuffer = 3;
  }
  return aux;
}
void setup(void) {

  Serial.begin(74880);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Start GS");

  lcd.setCursor(0, 1);
  lcd.print("Initialising");


  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  // turn on interrupts
  SPCR |= _BV(SPIE);

  SPIBufferPosition = 0;
  hasData = false;

  pinMode(wakeUpPin, OUTPUT);
  pinMode(radioStatusLightPin, OUTPUT);

  digitalWrite(wakeUpPin, HIGH);
  digitalWrite(radioStatusLightPin, LOW);

  //mS_sendMessage = prepareSerialMessage (groundStationName + "M",  groundStationName + "R", groundStationName + "R", testMessageSingle, 1, &characterBuffer[0]);
  //initialiseSPICommunicationOnSlave (wakeUpPin, mS_sendMessage);
  lcd.clear();
  while (!radioIsWorking)
  {
    if (!startingUp)
    {
      radioStartTries++;
      radioIsWorking = getRadioStatus();
      lcd.setCursor(0, 3);
      if (radioIsWorking)
      {
        lcd.print("Radio is operational");
        digitalWrite(radioStatusLightPin, HIGH);
      }
      else
      {
        lcd.print("Radio off : ");
        lcd.print(radioStartTries);
        waitFor(15000);
      }
    }
    else
    {
      startingUp = (millis() < 20000);
      waitFor(1000);
    }
  }
  nrChars = 0;
  _SERIAL_PRINTLN("Go Go Go");
}

void loop(void)
{
  if (hasData)
  {
    memcpy(&mR_receiveMessage, &byteMessage, sizeof(mR_receiveMessage));
    displayMessageLCD (&mR_receiveMessage);
    displayMessage(&mR_receiveMessage, false);
    hasData = false;
    SPIBufferPosition = 0;
    //
    // handle message from main computer
    //
    switch ((messageTypeIds)mR_receiveMessage.messageTypeId)
    {
      case radioStatusReply:
        if (mR_receiveMessage.content[0] == radioOn)
        {
          radioIsWorking = true;
          digitalWrite(radioStatusLightPin, HIGH);
          lcd.clear();
          lcd.setCursor(0, 3);
          lcd.print("Radio is operational");
        }
        else
        {
          radioIsWorking = false;
          digitalWrite(radioStatusLightPin, LOW);
          lcd.clear();
          lcd.setCursor(0, 3);
          lcd.print("Radio is not working");
        }
        memset (rawContent, 0, sizeof(rawContent));
        rawContent[0] = messageUnderstood;
        mS_sendMessage = prepareSerialMessage (groundStationName + "M", groundStationName + "R", groundStationName + "R", messageStatusReply, 1, &rawContent[0]);
        displayMessage(&mS_sendMessage, true);
        sendStatusMessageSPIFromMaster(wakeUpPin, mS_sendMessage);
        mR_receiveMessage.messageTypeId = 0; // noMessageActive
        break;
      case messageStatusReply:
        mR_receiveMessage.messageTypeId = 0; // noMessageActive
        break;
      default:
        mR_receiveMessage.messageTypeId = 0; // noMessageActive
        _SERIAL_PRINT("No valid function for Groundstation Mesh: ");
        _SERIAL_PRINTLN(mR_receiveMessage.messageTypeId);
        break;
    }
  }
#ifdef TEST_MODE
  if ((millis() - testWaitTimeStarted) > 60000)
  {
    testWaitTimeStarted = millis();
    newOrderFound = true;
    characterBuffer[0] = 65;
    characterBuffer[1] = 66;
    characterBuffer[2] = 90;
    characterBuffer[3] = 5;
    characterBuffer[4] = 2;
    characterBuffer[5] = 0;
    characterBuffer[6] = 38;
    characterBuffer[7] = 7;
    characterBuffer[8] = 0;
    characterBuffer[9] = 11;
    characterBuffer[10] = 0;
    characterBuffer[11] = 0;
    pointerInBuffer = 3;
    nrChars = 12;
  }
#else
  {
    if (Serial.available() > 0)
    {
      if (nrChars == 0)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        memset(characterBuffer, 0, sizeof(characterBuffer));
        if (flushUntilStartOfMessage())
        {
          nrChars = 3;
          while (Serial.available() > 0)
          {
            posNr++;
            if (posNr == 20)
            {
              posNr = 0;
              lineNr++;
              if (lineNr == 5)
              {
                posNr = 19;
              }
            }
            oneChar = Serial.read();
            lcd.setCursor(posNr, lineNr);
            lcd.print(oneChar + 32);
            characterBuffer[nrChars] = oneChar;
            nrChars++;
          }
          pointerInBuffer = 3;
          newOrderFound = true;
        }
        else
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("reset command?");
          while (Serial.available() > 0)
          {
            oneChar = Serial.read();
          }
          nrChars = 0;
          newOrderFound = false;
        }
      }
    }
  }
#endif


  if (newOrderFound)
  {
    messagesFromInput = messagesFromInput + 1;
    Serial.println("+=+=+=+");
    for (int i = 0; i < nrChars; i++)
    {
      Serial.print(characterBuffer[i]);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println("+=+=+=+");
    equipmentId      = characterBuffer[pointerInBuffer];
    equipmentName    = equipmentNameList[equipmentId].substring(0, 4);
    numberOfCommands = characterBuffer[pointerInBuffer + 1];
    pointerInBuffer  = 6;


    mS_sendMessage = prepareSerialMessage (groundStationName + "M",  groundStationName + "R", equipmentName + "M", vehicleCommandList, numberOfCommands, &characterBuffer[pointerInBuffer]);
    displayMessageLCD(&mS_sendMessage);
    for (int i = 0; i < numberOfCommands; i++)
    {
      positionRequestPending = positionRequestPending || (mS_sendMessage.content[i * 3] == requestPosition);
    }
    //    Serial.println();
    if (positionRequestPending)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("location req");
      requestForLocation(equipmentId);
      positionRequestPending = false;
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sending message");
      mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
      if ((mR_receiveMessage.messageTypeId == messageStatusReply) && (mR_receiveMessage.content[0] == messageUnknown))
      {
        lcd.print("Failure");
      }
      else
      {
        lcd.print("Succes");
      }
    }
    Serial.println("Go Go Go after message receive");
    nrChars = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("new command?");
    newOrderFound = false;
  }
}
