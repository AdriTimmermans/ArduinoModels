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
#include <EEPROM.h>
typedef struct  motorAttributesTemplate motorAttributesType;

#define triggerMasterInterruptPin 6

volatile int  globalPulseCount    = 0;
volatile bool pulseCountChanged   = false;
volatile int  timeFramePulseCount = 0;
bool currentSensorValue = LOW;
bool previousSensorValue = HIGH;

struct motorAttributesTemplate {
  byte motorId;
  byte motorTrackingSensorPin;
  byte motorPin1;
  byte motorPin2;
  byte motorSpeedPin;
  byte motorDirection;
  byte stepCount;
  byte motorSpeed;
  float targetRPM;
  float RPM;
  long lastRPMCalculation;
  int timerTime;
  int countWheel             = 0;
  int totalWheelPulses       = 0;
  int pulsesNeeded            = 0;
  int trackingSensorValue     = 0;
  int lastTrackingSensorValue = 0;
  int lastMotorSpeed          = -1;
  bool justChanged            = false;
  bool adjust                 = false;
  bool activeDisplay          = false;
  long currentOrderDuration   = 0;
  long currentOrderStartAt;

};
#define requestWheelPulsesCount 42
#define informMotorOrder        43
#define requestDisplayMotorData 44
#define informExecuteMotorOrder 45
byte typeOfRequest;
#define displayData             51
#define sendWheelPulsesCount    52
#define sendAckNack             53

typedef struct  motorMessageTemplate motorMessageType;
struct motorMessageTemplate {
  byte motorId;
  byte motorDirection;
  byte targetRPMFromMaster;
  byte currentOrderDuration;
  int pulsesNeeded;
};

volatile bool messageReceived = false;
volatile byte ackNackMotorMessage = 0;
const long timerTime = 3000;

class driveMotor
{

  public:
    motorAttributesType oneMotor;
    
    void begin (int EEPromStartAddress)
    {
      oneMotor.motorId                = EEPROM.read(EEPromStartAddress + 0);
      oneMotor.motorTrackingSensorPin = 2;//EEPROM.read(EEPromStartAddress + 1);
      oneMotor.motorSpeedPin          = EEPROM.read(EEPromStartAddress + 2);
      oneMotor.motorPin1              = EEPROM.read(EEPromStartAddress + 3);
      oneMotor.motorPin2              = EEPROM.read(EEPromStartAddress + 4);
      oneMotor.stepCount              = EEPROM.read(EEPromStartAddress + 5);
      oneMotor.countWheel             = 0;
      oneMotor.totalWheelPulses       = 0;
      oneMotor.trackingSensorValue    = 0;
      oneMotor.currentOrderDuration   = 0;
      oneMotor.motorDirection         = 0;
      oneMotor.RPM                    = 0;
      oneMotor.targetRPM              = 0;
      pinMode (oneMotor.motorTrackingSensorPin, INPUT);
      pinMode (oneMotor.motorSpeedPin, OUTPUT);
      pinMode (oneMotor.motorPin1, OUTPUT);
      pinMode (oneMotor.motorPin2, OUTPUT);
      digitalWrite(oneMotor.motorPin1, LOW);
      digitalWrite(oneMotor.motorPin2, LOW);
      analogWrite(oneMotor.motorSpeedPin, 0);
      _SERIAL_PRINT("Initialialisation: 0 send to speedpin");
      oneMotor.lastMotorSpeed         = 0;
      oneMotor.timerTime              = timerTime;
    }

    byte getPowerSettingForRPM(float targetRPM)
    {
      byte aux;
      byte i = 1;
      int EEPROMAddressBias;
      byte speedReference;

      (oneMotor.motorId < 4) ? EEPROMAddressBias = 400 : EEPROMAddressBias = 100;
      speedReference = EEPROM.read(EEPROMAddressBias);
      while (speedReference > (byte)targetRPM)
      {
        i++;
        speedReference = EEPROM.read(EEPROMAddressBias + i);
      }

      aux = 255 - ((i - 1) * 3);
      _SERIAL_PRINT("Speed ");
      _SERIAL_PRINT((byte)targetRPM);
      _SERIAL_PRINT(" found at power ");
      _SERIAL_PRINTLN(aux);
      aux = 255; // overrule and start motor at same (maximum) speed
      return aux;
    }

    void processMotorMessage (motorMessageType aMessage)
    {
      _SERIAL_PRINT("in ProcessMotorMessage for motor:");
      _SERIAL_PRINTLN(oneMotor.motorId);
      oneMotor.motorDirection         = aMessage.motorDirection;
      oneMotor.targetRPM              = (float)aMessage.targetRPMFromMaster;
      oneMotor.currentOrderDuration   = aMessage.currentOrderDuration;
      oneMotor.currentOrderStartAt    = millis();
      oneMotor.justChanged            = false;
      oneMotor.countWheel             = 0;
      oneMotor.totalWheelPulses       = 0;
      oneMotor.pulsesNeeded           = aMessage.pulsesNeeded;
      oneMotor.lastRPMCalculation     = 0;
      oneMotor.trackingSensorValue    = 0;
      oneMotor.RPM                    = 0;
      oneMotor.lastMotorSpeed         = 0;
      oneMotor.timerTime              = timerTime;
      if ((oneMotor.motorId == 2) && (oneMotor.motorDirection != 0))
      {
          previousSensorValue = digitalRead(oneMotor.motorTrackingSensorPin);
      }
      globalPulseCount    = 0;
      timeFramePulseCount = 0;
    }

    void determineTimerTime ()
    {
      int diff;
      diff = oneMotor.targetRPM - oneMotor.RPM;
      if (diff > 40)
      {
        oneMotor.timerTime = timerTime / 8;
      }
      else if (diff > 25)
      {
        oneMotor.timerTime = timerTime / 4;
      }
      else if (diff > 12)
      {
        oneMotor.timerTime = timerTime / 2;
      }
      else
      {
        oneMotor.timerTime = timerTime;
      }
      oneMotor.adjust = (diff > 7);
      oneMotor.adjust = false;
    }

    void triggerInterrupt (int intPin)
    {
      digitalWrite (intPin, HIGH); //
      delay(5); // 5 ms pulse
      digitalWrite (intPin, LOW); //
      _SERIAL_PRINTLN("Interrupt to master triggered");
    }

    void handleMotor ()
    {
      int adjustmentStep = 3;
      if (oneMotor.motorDirection != 0)
      {
        if (oneMotor.justChanged)
        {
          _SERIAL_PRINT("JustChanged: ");

          _SERIAL_PRINTLN(oneMotor.justChanged);
          if (oneMotor.motorDirection == 1)
          {
            digitalWrite(oneMotor.motorPin1, HIGH);
            digitalWrite(oneMotor.motorPin2, LOW);
            oneMotor.lastMotorSpeed = -1;
            oneMotor.motorSpeed = getPowerSettingForRPM(oneMotor.targetRPM);
            oneMotor.lastRPMCalculation = millis();
            _SERIAL_PRINT("Forwards");
          }
          else if (oneMotor.motorDirection == 2)
          {
            digitalWrite(oneMotor.motorPin1, LOW);
            digitalWrite(oneMotor.motorPin2, HIGH);
            oneMotor.lastMotorSpeed = -1;
            oneMotor.motorSpeed = getPowerSettingForRPM(oneMotor.targetRPM);
            oneMotor.lastRPMCalculation = millis();
            _SERIAL_PRINT("Backwards");
          }
          else
          {
            digitalWrite(oneMotor.motorPin1, LOW);
            digitalWrite(oneMotor.motorPin2, LOW);
            oneMotor.motorSpeed = 0;
            oneMotor.lastRPMCalculation = 0;
            _SERIAL_PRINT("\t Stop");
            _SERIAL_PRINT(oneMotor.motorDirection);
          }
          _SERIAL_PRINT("\t motorSpeed: ");
          _SERIAL_PRINTLN(oneMotor.motorSpeed);
          oneMotor.justChanged = false;
        }
        if (oneMotor.adjust)
        {
          _SERIAL_PRINT("Motor: ");
          _SERIAL_PRINT(oneMotor.motorId);
          _SERIAL_PRINT("\t TargetRPM: ");
          _SERIAL_PRINT(oneMotor.targetRPM);
          _SERIAL_PRINT("\t RPM ");
          _SERIAL_PRINT(oneMotor.RPM);
          oneMotor.motorSpeed = constrain(oneMotor.motorSpeed, getPowerSettingForRPM(7.0) + 1 + adjustmentStep, 255 - 1 - adjustmentStep);
          ((oneMotor.targetRPM - oneMotor.RPM) > 0) ? oneMotor.motorSpeed = oneMotor.motorSpeed + adjustmentStep : oneMotor.motorSpeed = oneMotor.motorSpeed - adjustmentStep;
          _SERIAL_PRINT("\t motorSpeed=");
          _SERIAL_PRINTLN(oneMotor.motorSpeed);
          oneMotor.adjust = false;
        }

        if (oneMotor.currentOrderDuration != 0)
        {
          if ((millis() - oneMotor.currentOrderStartAt) >= oneMotor.currentOrderDuration * 1000)
          {
            _SERIAL_PRINTLN(" == Run duration elapsed");
            digitalWrite(oneMotor.motorPin1, LOW);
            digitalWrite(oneMotor.motorPin2, LOW);
            oneMotor.motorDirection       = 0;
            oneMotor.targetRPM            = 0;
            oneMotor.currentOrderDuration = 0;
            oneMotor.motorSpeed           = 0;
            oneMotor.lastMotorSpeed       = -1;
            oneMotor.lastRPMCalculation   = 0;
          }
        }

        if (oneMotor.lastMotorSpeed != oneMotor.motorSpeed)
        {
          _SERIAL_PRINT("running lastmotorspeed != motorspeed: ");
          _SERIAL_PRINTLN(oneMotor.motorSpeed);
          analogWrite(oneMotor.motorSpeedPin, oneMotor.motorSpeed);
          oneMotor.lastMotorSpeed = oneMotor.motorSpeed;
        }
        if (oneMotor.motorId == 2)
        {

          currentSensorValue = digitalRead(oneMotor.motorTrackingSensorPin);
          pulseCountChanged = (previousSensorValue != currentSensorValue);
          previousSensorValue = currentSensorValue;
          if (pulseCountChanged)
          {
            timeFramePulseCount++;
            globalPulseCount++;
            oneMotor.countWheel = timeFramePulseCount;
            oneMotor.totalWheelPulses = globalPulseCount;
            if ((oneMotor.totalWheelPulses >= oneMotor.pulsesNeeded))
            {
              _SERIAL_PRINTLN("Should stop now...");
              triggerInterrupt(triggerMasterInterruptPin);
              detachInterrupt(digitalPinToInterrupt(oneMotor.motorTrackingSensorPin));
              oneMotor.totalWheelPulses = oneMotor.pulsesNeeded - 1;
            }
          }
        }
      }
      else
      {
        if (oneMotor.justChanged)
        {
          digitalWrite(oneMotor.motorPin1, LOW);
          digitalWrite(oneMotor.motorPin2, LOW);
          oneMotor.motorSpeed = 0;
          oneMotor.lastRPMCalculation = 0;
          _SERIAL_PRINTLN ("stop order interpreted, send 0 to motorspeed");
          analogWrite(oneMotor.motorSpeedPin, oneMotor.motorSpeed);
          oneMotor.lastMotorSpeed = 0;
          oneMotor.justChanged = false;
          oneMotor.totalWheelPulses = 0;
        }
      }
    }

    void serialDisplay()
    {
      _SERIAL_PRINT("Motor: ");
      _SERIAL_PRINT(oneMotor.motorId);
      _SERIAL_PRINT("\t Direction: ");
      _SERIAL_PRINT(oneMotor.motorDirection);
      _SERIAL_PRINT("\t targetRPM: ");
      _SERIAL_PRINT(oneMotor.targetRPM);
      _SERIAL_PRINT("\t pulsesNeeded: ");
      _SERIAL_PRINT(oneMotor.pulsesNeeded);
      _SERIAL_PRINT("\t Order duration: ");
      _SERIAL_PRINT(oneMotor.currentOrderDuration * 1000);
      _SERIAL_PRINT("\t currentOrderStartAt: ");
      _SERIAL_PRINTLN(oneMotor.currentOrderStartAt);
    }

};

driveMotor motorLeftSide, motorRightSide;

motorMessageType oneMessage;

//#define SDA A4
//#define SCL A5
long numberOfInterrupts = 0;
byte targetMotorIDOfMessage;

#define minDelay 100
#define maxDelay 1000

void manualTimer()
{
  if (motorLeftSide.oneMotor.lastRPMCalculation > 0)
  {
    if ((millis() - motorLeftSide.oneMotor.lastRPMCalculation) > motorLeftSide.oneMotor.timerTime)
    {
      _SERIAL_PRINT(motorLeftSide.oneMotor.countWheel);
      _SERIAL_PRINT(" / ");
      _SERIAL_PRINT(motorLeftSide.oneMotor.stepCount);
      _SERIAL_PRINT(" => ");
      _SERIAL_PRINTLN(((float)motorLeftSide.oneMotor.countWheel / (float)motorLeftSide.oneMotor.stepCount));
      (motorLeftSide.oneMotor.countWheel > 0) ? motorLeftSide.oneMotor.RPM = (60000.0 / (float)timerTime) * ((float)motorLeftSide.oneMotor.countWheel / (float)motorLeftSide.oneMotor.stepCount) : motorLeftSide.oneMotor.RPM = motorLeftSide.oneMotor.targetRPM;
      _SERIAL_PRINT("Left pulseCount = ");
      _SERIAL_PRINT(motorLeftSide.oneMotor.countWheel);
      _SERIAL_PRINT(", RPM = ");
      _SERIAL_PRINTLN(motorLeftSide.oneMotor.RPM);
      motorLeftSide.oneMotor.lastRPMCalculation = millis();
      motorLeftSide.determineTimerTime();   // adjust update speed and value
      motorLeftSide.oneMotor.countWheel = 0;
    }
  }

  if (motorRightSide.oneMotor.lastRPMCalculation > 0)
  {
    if ((millis() - motorRightSide.oneMotor.lastRPMCalculation) > motorRightSide.oneMotor.timerTime)
    {
      _SERIAL_PRINT(motorRightSide.oneMotor.countWheel);
      _SERIAL_PRINT(" / ");
      _SERIAL_PRINT(motorRightSide.oneMotor.stepCount);
      _SERIAL_PRINT(" => ");
      _SERIAL_PRINTLN(((float)motorRightSide.oneMotor.countWheel / (float)motorRightSide.oneMotor.stepCount));
      (motorRightSide.oneMotor.countWheel > 0) ? motorRightSide.oneMotor.RPM = (60000.0 / (float)timerTime) * ((float)motorRightSide.oneMotor.countWheel / (float)motorRightSide.oneMotor.stepCount) : motorRightSide.oneMotor.RPM = motorRightSide.oneMotor.targetRPM;
      _SERIAL_PRINT("Right pulseCount = ");
      _SERIAL_PRINT(motorRightSide.oneMotor.countWheel);
      _SERIAL_PRINT(", RPM = ");
      _SERIAL_PRINTLN(motorRightSide.oneMotor.RPM);
      motorRightSide.oneMotor.lastRPMCalculation = millis();
      motorRightSide.determineTimerTime();   // adjust update speed and value
      motorRightSide.oneMotor.countWheel = 0;
    }
  }
}

void setup()
{
  int pointer = 0;
  int slaveID = EEPROM.read(0);

  _SERIAL_BEGIN(9600);
  pinMode(triggerMasterInterruptPin, OUTPUT);
  _SERIAL_PRINT("ID of Device: ");
  _SERIAL_PRINTLN(slaveID);

  delay(random(minDelay, maxDelay)); // to prevent starting up the connection to I2C at exactly the same time
  for (int i = 255; i > 0; i = i - 3)
  {
    _SERIAL_PRINT (i);
    _SERIAL_PRINT("\t ");
    _SERIAL_PRINT(EEPROM.read(100 + pointer));
    _SERIAL_PRINT("\t ");
    _SERIAL_PRINTLN(EEPROM.read(400 + pointer));
    pointer++;
  }

  Wire.begin(slaveID);
  _SERIAL_PRINTLN("Wire.begin executed: ");
  Wire.onReceive(receiveMasterMessage);
  Wire.onRequest(sendMasterMessage);

  motorLeftSide.begin(1);
  motorLeftSide.serialDisplay();
  _SERIAL_PRINT(", pin1: ");
  _SERIAL_PRINT(motorLeftSide.oneMotor.motorPin1);
  _SERIAL_PRINT(", pin2: ");
  _SERIAL_PRINT(motorLeftSide.oneMotor.motorPin2);
  _SERIAL_PRINT(", speedpin: ");
  _SERIAL_PRINTLN(motorLeftSide.oneMotor.motorSpeedPin);
  motorRightSide.begin(11);
  motorRightSide.serialDisplay();
  _SERIAL_PRINT(", pin1: ");
  _SERIAL_PRINT(motorRightSide.oneMotor.motorPin1);
  _SERIAL_PRINT(", pin2: ");
  _SERIAL_PRINT(motorRightSide.oneMotor.motorPin2);
  _SERIAL_PRINT(", speedpin: ");
  _SERIAL_PRINTLN(motorRightSide.oneMotor.motorSpeedPin);

}

void sendMasterMessage()
{
  char line[32];

  switch (typeOfRequest)
  {
    case displayData:
      sprintf(line, "%1d %3d %3d-%1d %3d %3d ",
              motorLeftSide.oneMotor.motorDirection,
              (int)motorLeftSide.oneMotor.targetRPM,
              (int)motorLeftSide.oneMotor.RPM,
              motorRightSide.oneMotor.motorDirection,
              (int)motorRightSide.oneMotor.targetRPM,
              (int)motorRightSide.oneMotor.RPM);
      Wire.write(line);
      _SERIAL_PRINTLN(line);
      break;
    case sendWheelPulsesCount:
      Wire.write(((motorLeftSide.oneMotor.totalWheelPulses & 0xFF00) >> 8));
      Wire.write((motorLeftSide.oneMotor.totalWheelPulses & 0x00FF));
      _SERIAL_PRINT("Pulserequest received, count = ");
      _SERIAL_PRINTLN(motorLeftSide.oneMotor.totalWheelPulses);
      break;
    case sendAckNack:
      //Wire.write(ackNackMotorMessage);
      //_SERIAL_PRINT("ackNackMotorMessage sent = ");
      //_SERIAL_PRINTLN(ackNackMotorMessage);
      break;

  }

}

void receiveMasterMessage(int howMany)
{

  byte messageTypeForMotor;
  byte pulsesNeededMSB;
  byte pulsesNeededLSB;
  byte CRCCheckValue;
  int  CRCValue;

  Serial.print("In receiveMasterMessage - messageTypeForMotor: ");
  messageTypeForMotor = Wire.read();
  Serial.println(messageTypeForMotor);
  switch (messageTypeForMotor)
  {
    case informMotorOrder :
      oneMessage.motorId = Wire.read();
      oneMessage.motorDirection = Wire.read();
      oneMessage.targetRPMFromMaster = Wire.read();
      oneMessage.currentOrderDuration = Wire.read();
      pulsesNeededMSB = Wire.read();
      pulsesNeededLSB = Wire.read();
      CRCValue = oneMessage.motorId + oneMessage.motorDirection + oneMessage.targetRPMFromMaster + oneMessage.currentOrderDuration + pulsesNeededMSB + pulsesNeededLSB;
      CRCCheckValue = Wire.read();
      ((CRCValue & 0x00FF) == CRCCheckValue) ? ackNackMotorMessage = 1 : ackNackMotorMessage = 0;
      oneMessage.pulsesNeeded = (pulsesNeededMSB << 8) + pulsesNeededLSB;
      if (oneMessage.pulsesNeeded <= 0)
      {
        oneMessage.pulsesNeeded = 9999; // no distance set means motor only stops after 100 meters (or if stopped by other means)
      }
      if (ackNackMotorMessage == 1)
      {
        (oneMessage.motorId < 4) ? motorLeftSide.processMotorMessage(oneMessage) : motorRightSide.processMotorMessage(oneMessage);
      }
      messageReceived = true;

      typeOfRequest = sendAckNack;
      break;
    case requestDisplayMotorData:
      typeOfRequest = displayData;
      break;
    case requestWheelPulsesCount:
      typeOfRequest = sendWheelPulsesCount;
      break;
    case informExecuteMotorOrder:
      motorRightSide.oneMotor.justChanged = true;
      motorLeftSide.oneMotor.justChanged = true;
      break;
  }

}

void loop()
{


  if (messageReceived)
  {
    motorLeftSide.serialDisplay();
    motorRightSide.serialDisplay();
    messageReceived = false;
  }
  //manualTimer();
  motorLeftSide.handleMotor();
  motorRightSide.handleMotor();
  delay(30);
}
