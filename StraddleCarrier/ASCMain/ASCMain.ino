// This is the master computer on the A-Straddecarrier
//#define TEST_MODE 1

#ifdef TEST_MODE
int delayFactor = 1;
#else
int delayFactor = 0;
#endif
#include<EEPROM.h>
#include <SPI.h>
#include <Wire.h> // Using I2C communication to communicate between Arduino's on board
#include <LiquidCrystal_I2C.h>
#include<ASCCommunicationDefinition.h>
#include<ASCModelParameters.h>
#include <LinkedList.h>
#include <Math.h>
#include <EEPROMTEXT.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>
#include <LSM303.h>
#include <Adafruit_PWMServoDriver.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};
#include <DS3231.h>

DS3231 RTC;
RTClib defineNow;
bool Century = false;
bool h12 = true;
bool PM = true;

//
// I2C pins above pin 13 to free the pin 20 for an interrupt
//
// define pins
#define interruptFromSlavePin           2
#define interruptSpreaderFinishPin      3
#define motorSpreaderEnablePin          4
#define steeringMotorPowerSwitch        6
#define masterReadyForSlavePin         10
#define wakeUpPin                      18
#define interruptStopMainMotorsPin     19
#define motorSpreaderPin1              22
#define motorSpreaderPin2              23
#define spreaderLockPin                24
#define callFromInterruptPin           25
#define callToI2CPin                   26
#define waitForAnswerPin               27
#define poleMotorPin1                  30    // 28BYJ48 pin 1
#define poleMotorPin2                  31    // 28BYJ48 pin 2
#define poleMotorPin3                  32     // 28BYJ48 pin 3
#define poleMotorPin4                  33    // 28BYJ48 pin 4
#define forwardIndicatorPin2           34
#define backwardIndicatorPin2          35
#define spreaderLightsPin              36
#define laserPointerPin                37    // used to visualize where the ToF-sensor is pointing at
#define startupCloseDownSlave          40    // used to synchronise starting of master and slave arduino
#define startupCloseDownMotors         41    // used to synchronise starting of master and ArduinoProMinis
#define connectI2CSlave                42    // used to connect or disconnect I2C wires when Slave is not powered on
#define connectI2CMotors               43    // used to connect or disconnect I2C wires when ArduinoProMinis are not powered on
#define buzzerPin                      44
#define lightSourcePin                 45
#define radioActiveLightPin            46
#define lightSensorPin                 A0
#define counterMainMotorSensor         A4 // connected to counter in left middle wheel, not used at the moment (april 2021)
#define rightFollowLineSensorPin       A5
#define leftFollowLineSensorPin        A6
#define compassMotorPin1               A8    // 28BYJ48 pin 1
#define compassMotorPin2               A9    // 28BYJ48 pin 2
#define compassMotorPin3              A10     // 28BYJ48 pin 3
#define compassMotorPin4              A11    // 28BYJ48 pin 4
#define motorInterfaceType              1
#define slaveNodeAddress              0x09

enum matrixBoardIcons
{
  suls = 0,
  sulf = 1,
  sulb = 2,
  sulr = 3,
  sull = 4,
  suus = 5,
  suuf = 6,
  suub = 7,
  suur = 8,
  suul = 9,
  sdls = 10,
  sdlf = 11,
  sdlb = 12,
  sdlr = 13,
  sdll = 14,
  sdus = 15,
  sduf = 16,
  sdub = 17,
  sdur = 18,
  sdul = 19
};

byte s[20][9] =
{ {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x3C, 0x3C, 0x00, 0x00, 0x00},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x3C, 0x3C, 0x00, 0x18, 0x24},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x3C, 0x3C, 0x00, 0x24, 0x18},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x3C, 0x3C, 0x02, 0x01, 0x02},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x3C, 0x3C, 0x40, 0x80, 0x40},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x00, 0x00, 0x00, 0x18, 0x24},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x00, 0x00, 0x00, 0x24, 0x18},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x00, 0x00, 0x08, 0x04, 0x08},
  {informMatrixBoardData, 0x42, 0x42, 0xFF, 0x00, 0x00, 0x10, 0x20, 0x10},
  {informMatrixBoardData, 0x00, 0x00, 0x00, 0x42, 0x42, 0xFF, 0x3C, 0x3C},
  {informMatrixBoardData, 0x18, 0x24, 0x00, 0x42, 0x42, 0xFF, 0x3C, 0x3C},
  {informMatrixBoardData, 0x24, 0x18, 0x00, 0x42, 0x42, 0xFF, 0x3C, 0x3C},
  {informMatrixBoardData, 0x08, 0x04, 0x08, 0x42, 0x42, 0xFF, 0x3C, 0x3C},
  {informMatrixBoardData, 0x10, 0x20, 0x10, 0x42, 0x42, 0xFF, 0x3C, 0x3C},
  {informMatrixBoardData, 0x00, 0x00, 0x00, 0x42, 0x42, 0xFF, 0x00, 0x00},
  {informMatrixBoardData, 0x18, 0x24, 0x00, 0x42, 0x42, 0xFF, 0x00, 0x00},
  {informMatrixBoardData, 0x24, 0x18, 0x00, 0x42, 0x42, 0xFF, 0x00, 0x00},
  {informMatrixBoardData, 0x08, 0x04, 0x08, 0x42, 0x42, 0xFF, 0x00, 0x00},
  {informMatrixBoardData, 0x10, 0x20, 0x10, 0x42, 0x42, 0xFF, 0x00, 0x00}
};

const uint8_t mainI2CChannel        = 1;
const uint8_t miniProEEPROMChannel  = 6;
uint8_t       selectedI2CChannel    = 0;
const byte EEPROMI2CAddress         = 0x50;
const byte DASHBOARD_ADDRESS        = 0x0E;
//const byte EEPROMCSPin            = 53;

EEPROMText EETexts(miniProEEPROMChannel, EEPROMI2CAddress, EEPROMI2CAddress, mainI2CChannel);

VL53L1X toFSensor;

#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2400
#define FREQUENCY 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

//int servoDelay(5);

bool slaveInitialising = false;
bool reDisplay = false;
bool sentSuccess = false;
bool radioIsWorking = false;

// define vehicle params
String straddleName      = equipmentNameList[1].substring(0, 4); // Default is the first equipment in use, should come from calculations of ground station
String groundStationName = equipmentNameList[0].substring(0, 4);
String pointZeroName     = equipmentNameList[11].substring(0, 4); // Name of originPole
byte executingStop  = 0;
byte executingMove  = 1;
byte executingRight = 2;
byte executingLeft = 3;

int executionStatus = 0;
bool spreaderOccupied = false;
bool vehicleMoving = false;

volatile bool checkSpreaderLights = false;
volatile bool checkMessageTimeOut = false;
volatile bool checkWheelPulseTimeOut = false;
volatile bool checkSpreaderHeight = false;
volatile bool askForPulsesPassed = false;
volatile bool askForSpreaderHeight = false;
volatile bool checkI2CReturn      = false;
volatile int timeOutI2CCount = 0;
volatile long lastPulseCheck;
volatile bool originFoundIndicator = false;

enum tofSensorStatusValues
{
  operational,
  failure,
  notInitialised
};

tofSensorStatusValues tofSensorStatus = notInitialised;

bool operationsActiveMode = false;

volatile bool triggerReceivedFromSlave = false;

aMessage mR_receiveMessage;
aMessage mS_sendMessage;

#define stopped 0
#define slow 90
#define fast 100

#define maxTurntime 5000
#define marginOfErrorInAngle 10 // degrees
#define marginOfErrorInGrid 15  // cm, later we need to do exact positioning in another method

// parameters for location determination

byte drivingDirection = drivingForwards;

int andCounting = 0;

int mainStop = 0;
int maxRPM = 99;
int steeringStop = 0;
int steeringMax;
float wheelDiameter = 3.40;
int pulsesPerWheelRotation = 16;
const float maxWheelAmplitude = 20.0;
float distancePerPulse = 2 * M_PI * wheelDiameter / pulsesPerWheelRotation;

//
// Calculations:
// Wheelbase : 275 mm
// wheel distance front/rear wheels: 300 mm
// Turn arc = 20 degrees of inside wheel = maxWheelAmplitude
//
// ==> radius of arc of inside wheel => r(inside) * sin(20) = 150 mm => r(inside) = 438.5 mm
// ==> radius of arc of outside wheel = r(inside) + 275 mm
// ==> arc of outside wheel = arcsin (150mm/(r(outside)) = arcsin(150/713.5) = 12.1 degrees
// ==> ratio arc(outside wheel)/arc(inside wheel) = 12.1/20 = 0.61
//
// speed of wheels:
//
// diameter of wheels = 34 mm => 1 rotation = 213.6 mm
// distance inside wheel = r(inside) * 2 * pi = 2755.1 mm
// rotations inside wheel = 2755.1/213.6 = 12.8 rotations
// distance outside wheel = r(outside) * 2 * pi = 4483.0 mm
// rotations outside wheel = 4483.0 / 213.6 = 21.0 rotations
//
// ratio (speed(inside wheel) / speed(outside wheel)) = 12.8 / 21 = 0.61
float differentialTurnFraction    = 0.65; // differential = 1 at straight driving, at left-turn : the left wheel should cover 0.61 x distance of right wheels

typedef struct motorMotionOrderTemplate motorMotionOrderType;
struct motorMotionOrderTemplate
{
  byte slaveId;         // I2C id of Arduino Pro Mini used for motion motors (2 motors per MC and H-Bridge)
  byte motorId;         // left side = 1,2 and 3. Right side = 4, 5 and 6
  byte direction;       // 1 = forward, 2 = backward, o = stop
  byte rpm;             // speed in rotations per minute
  byte duration;        // number of tenth of seconds to run
  int  pulsesNeeded;    // int with required pulsed MSB
  byte CRC;             //
};
motorMotionOrderType motionMotorOrder;

typedef struct lcdMeta lcdParams;
struct lcdMeta
{
  byte columns;
  byte rows;
};

lcdParams lcdMotorsParams;
lcdParams lcdMasterParams;
lcdParams lcdSlaveParams;

volatile bool spreaderIsMoving = false;
bool orderListDisplaySuspended = false;
long  duration;
long  distance;
//volatile operationPossibilities activeOperationalState = switchedOffMode;
operationPossibilities inputOS;

// messageTypeIds  vehicleCommunicationModuleMessageId, vehicleMainComputerMessageId;
messageCommands vehicleComminicationMessageCommand,  vehicleMainComputerMessageCommand;

String   messageAcknowledged;

// Define I2c parameters


byte masterReceive;

// Define LCD pinout

const int i2cLCDMasterAddress = 0x23;
const int i2cLCDMotorsAddress = 0x25;
const int i2cLCDSlaveAddress = 0x27;

// Create new LCD instance

LiquidCrystal_I2C lcdMotors(i2cLCDMotorsAddress, 20, 4);
LiquidCrystal_I2C lcdMaster(i2cLCDMasterAddress, 16, 2);
LiquidCrystal_I2C lcdSlave(i2cLCDSlaveAddress, 16, 2);

long lastMotorDisplayTime;
int  displayMotorRefreshTime = 2000;
unsigned int runNumber;

char previousLogLine[80] = " ";


// define pole motor parameters

#define clockwise  0
#define anticlockwise 1

int poleMotorcount = 0;          // count of steps made
int poleMotorOrientationRelativeToVehicle = 0; // Time of Fliht sensor starts always towards the front-middel of the vehicle
int poleMotorcurrentdirection = anticlockwise;
int poleRequiredrotation = 0;

// define steering motors parameters
#define neutral 0

// define hoist motors parameters

#define spreaderPositionHigh  0
#define spreaderPositionmiddle 180
#define spreaderPositionLow 360

bool extendedSetup = true;

volatile bool spreaderMotorOn = false;
volatile bool mainMotorsOn = false;

int spreaderCurrentPosition = 0;
int spreaderCurrentDirection = neutral;
int hoistAnglechange = 0;

volatile bool spreaderLightIsOn = false;

const int stepsPerRevolution = 512;

int countsperrev = 512; // number of steps per full revolution
int lookup[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};

float uniPolarstepperMotoronedegree = (float)stepsPerRevolution / 360.0;
float uniPolarstepperMotorDegreesPerStep = 360.0 / (float)stepsPerRevolution;

float lastKnownlocationX = 0.0;
float lastKnownlocationY = 0.0;
float distanceBetweenlasers = 1.0; // in meters

bool  debugMode = true;
int   rightMotorRPM = 0;
int   testRuns = 0;
volatile float distanceTraveled;
volatile operationPossibilities activeOperationalState = standByMode;
int   returntoStatus;
operationPossibilities previousActiveOperationalState = abortedRunMode;
//
// dashboardInfo pushes the value to display on the vehicle dashboard to the dashboard slave
//
int   dashboardInfo;
int   dashboardTestinfo = 54;

// compass initialisation

const bool LEDOn = true;
const bool LEDOff = false;

// motor rotation sensor

volatile int   previousMainMotorSensorValue = 0;
int   currentMainMotorSensorValue = 0;
byte  mode = 0x02; //mode select register
byte  reg = 0x00; //continuous read mode
byte  xreg = 0x03; //first data register (1 of 6, MSB and LSB for x, y and z

float initialHeading;
float currentHeading;
float targetHeading;
float orderedHeading;
int   orderedDistance;
float compareHeading;
double calculatedHeading;

char *dtostrf(double val, signed char width, unsigned char prec, char *s);

int paramToGo;
float distanceToComplete;

gridPositionType currentPosition;

// matches values on master side.

bool executingRightTurn = false;
bool executingLeftTurn = false;
//
// route list structures
//
typedef struct poleCoordinateType poleCoordinates;
struct poleCoordinateType
{
  int coordinateAngle;
  int coordinateDistance;
};
int distanceInMM = 0;
float originAngleRelativeToVehicle = 0.0;
float originAngleRelativeToGrid = 0.0;
float poleMotorAngle = 0.0;
float orientationOfGridRelativeToNorth = 23.0 ; // in studeerkamer op de opera Zwijndrecht
float orientationVehicleRelativeToNorth = 23.0; // SC gepositioneerd in de positieve richting op de x-as
float correctionOfLaserposition = 0; // degrees off from true straight as a result of a wider field of vision

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

LinkedList<equipmentOrderType> equipmentOrders = LinkedList<equipmentOrderType>();

// routesegement parameters

bool skipRestOfRouteInput = false;
long timeoutThreshold = 100000;
long lastMessageFoundAt;
bool endOfRouteList = false;
int  mainLoopDelay = 10;

int routeListNodeNumber;
equipmentOrderType currentEquipmentOrder;
//

volatile int minimumFreeRam = 8192;

void triggerFromSlave()
{
  setCommunicationLED(callFromInterruptPin, LEDOn, 0);
  triggerReceivedFromSlave = true;
}

void compassMotorAntiClockWise(int rotateDegrees)
{
  int aux = (float)rotateDegrees * uniPolarstepperMotoronedegree;

  for (int j = 0; j < aux; ++j)
  {
    for (int i = 7; i >= 0; i--)
    {
      setOutputCompass(i);
      delayMicroseconds(compassMotorSpeed);
    }
  }
  digitalWrite(compassMotorPin1, LOW);
  digitalWrite(compassMotorPin2, LOW);
  digitalWrite(compassMotorPin3, LOW);
  digitalWrite(compassMotorPin4, LOW);
}
void setOutputCompass(int out)
{
  (bitRead(lookup[out], 0) == 1) ? digitalWrite(compassMotorPin1, HIGH) : digitalWrite(compassMotorPin1, LOW);
  (bitRead(lookup[out], 1) == 1) ? digitalWrite(compassMotorPin2, HIGH) : digitalWrite(compassMotorPin2, LOW);
  (bitRead(lookup[out], 2) == 1) ? digitalWrite(compassMotorPin3, HIGH) : digitalWrite(compassMotorPin3, LOW);
  (bitRead(lookup[out], 3) == 1) ? digitalWrite(compassMotorPin4, HIGH) : digitalWrite(compassMotorPin4, LOW);
}

void compassMotorClockWise(int rotateDegrees)
{
  int aux = (float)rotateDegrees * uniPolarstepperMotoronedegree;

  for (int j = 0; j < aux; ++j)
  {
    for (int i = 0; i < 8; ++i)
    {
      setOutputCompass(i);
      delayMicroseconds(compassMotorSpeed);
    }
  }
  digitalWrite(compassMotorPin1, LOW);
  digitalWrite(compassMotorPin2, LOW);
  digitalWrite(compassMotorPin3, LOW);
  digitalWrite(compassMotorPin4, LOW);

}

void setServoToAngle (int servoMotorId, int motorAngle)
{
  int pulseRAW;
  int calibratedMotorAngle;
  int pulseWidth;
  //
  // position on PCA965 per possible wheelselection (including 0 = no wheel selected
  //
  const int PCA9685[7] = {0, 0, 0, 8, 9, 0, 12};

  TCA9548A(miniProEEPROMChannel);
  calibratedMotorAngle = motorAngle + steeringCalibration[servoMotorId];
  pulseRAW = map(calibratedMotorAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulseWidth = int(float(pulseRAW) / 1000000 * FREQUENCY * 4096);
  pwm.setPWM(PCA9685[servoMotorId], 0, pulseWidth);
  delay(50);

}

void initCompass()
{
  TCA9548A(mainI2CChannel);
  if (compass.init())
  {
    compass.enableDefault();
    delay(1000);

    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < 360; i = i + 5) //calibration
      {
        ((j % 2) == 0) ? compassMotorClockWise(5) : compassMotorAntiClockWise(5);
        compass.read();
        delay(50);
        running_min.x = min(running_min.x, compass.m.x);
        running_min.y = min(running_min.y, compass.m.y);
        running_min.z = min(running_min.z, compass.m.z);

        running_max.x = max(running_max.x, compass.m.x);
        running_max.y = max(running_max.y, compass.m.y);
        running_max.z = max(running_max.z, compass.m.z);
      }
    }

    compass.m_min = running_min;
    compass.m_max = running_max;

    _SERIAL_PRINT("Calibration data (min): ");
    _SERIAL_PRINT(running_min.x);
    _SERIAL_PRINT("\t ");
    _SERIAL_PRINT(running_min.y);
    _SERIAL_PRINT("\t ");
    _SERIAL_PRINT(running_min.z);
    _SERIAL_PRINTLN("");
    _SERIAL_PRINT("Calibration data (max): ");
    _SERIAL_PRINT(running_max.x);
    _SERIAL_PRINT("\t ");
    _SERIAL_PRINT(running_max.y);
    _SERIAL_PRINT("\t ");
    _SERIAL_PRINT(running_max.z);
    _SERIAL_PRINTLN("");
    currentHeading = getHeading();
    debugSerialPrint(121, 121, false);//("--Initial Heading: ";
    _SERIAL_PRINTLN(currentHeading);
  }
  else
  {
    soundAlarm(3, 1000);
    while (1) {};
  }
}
void soundAlarm (byte numberOfBeeps, int soundDuration)
{
  _SERIAL_PRINTLN("Should hear alarm");
  for (int i = 0; i < numberOfBeeps; i++)
  {
    tone(buzzerPin, 1000); // Send 1KHz sound signal...
    delay(soundDuration);        // ...for .5 sec
    noTone(buzzerPin);     // Stop sound...
    delay(soundDuration);        // ...for .5 sec
  }
}

void soundSiren (int soundDuration)
{
  long startTime;
  startTime = millis();

  _SERIAL_PRINTLN("Should hear siren");
  while ((millis() - startTime) < soundDuration)
  {
    for (int i = 500; i < 1000; i++)
    {
      if ((millis() - startTime) < soundDuration)
      {
        tone(buzzerPin, i); // Send modulating sound signal...
        delay(5);        // ...for .5 sec
      }
    }
    for (int i = 1000; i < 500; i--)
    {
      if ((millis() - startTime) < soundDuration)
      {
        tone(buzzerPin, i); // Send 1KHz sound signal...
        delay(5);        // ...for .5 sec
      }
    }
  }
  noTone(buzzerPin);
}


void TCA9548A(uint8_t bus)
{
  if (selectedI2CChannel != bus)
  {
    Wire.beginTransmission(0x70);
    Wire.write(1 << bus);
    Wire.endTransmission();
    waitFor(40);
    selectedI2CChannel = bus;
  }
}

void transmitI2CMessage(byte I2CChannel)
{
  int CRCValue;
  byte ackNack;

  waitFor(50);
  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(I2CChannel);
  CRCValue =  motionMotorOrder.motorId + motionMotorOrder.direction + motionMotorOrder.rpm + motionMotorOrder.duration + ((motionMotorOrder.pulsesNeeded & 0xFF00) >> 8) + (motionMotorOrder.pulsesNeeded & 0x00FF);
  ackNack = 0;
  //  do
  //  {
  Wire.beginTransmission(motionMotorOrder.slaveId); // transmit to device
  Wire.write(informMotorOrder);
  Wire.write(motionMotorOrder.motorId);
  Wire.write(motionMotorOrder.direction);
  Wire.write(motionMotorOrder.rpm);
  Wire.write(motionMotorOrder.duration);
  Wire.write(((motionMotorOrder.pulsesNeeded & 0xFF00) >> 8));
  Wire.write((motionMotorOrder.pulsesNeeded & 0x00FF));
  Wire.write((CRCValue & 0x00FF));
  Wire.endTransmission();    // stop transmitting
  //Wire.requestFrom((int)motionMotorOrder.slaveId, 1);
  //ackNack = Wire.read();
  //  }
  //  while (ackNack != 1);
  setCommunicationLED(callToI2CPin, LEDOff, 0);
}

void executeMotorOrders()
{

  waitFor(50);
  TCA9548A(miniProEEPROMChannel);
  setCommunicationLED(callToI2CPin, LEDOn, 10);

  Wire.beginTransmission(0x0B); // transmit to device
  Wire.write(informExecuteMotorOrder);
  Wire.endTransmission();    // stop transmitting
  delay(20);
  Wire.beginTransmission(0x0D); // transmit to device
  Wire.write(informExecuteMotorOrder);
  Wire.endTransmission();    // stop transmitting
  delay(20);
  Wire.beginTransmission(0x0C); // transmit to device
  Wire.write(informExecuteMotorOrder);
  Wire.endTransmission();    // stop transmitting

  setCommunicationLED(callToI2CPin, LEDOff, 0);
}

void sendMotorOrdersLeftTurn (byte directionIndicator, byte reactionTimeOut, byte basisRPM)
{
  // basisRPM to be used later, now vehicle always drives at maxRPM
  float aux;
  byte rightWheelsSpeed  = maxRPM;  // outer wheel
  aux = differentialTurnFraction * (float) maxRPM;    // inner wheel
  byte leftWheelsSpeed  = (byte) aux;

  transmitI2CLeftMotorMessageBlock (directionIndicator, reactionTimeOut, leftWheelsSpeed, 0);
  transmitI2CRightMotorMessageBlock (directionIndicator, reactionTimeOut, rightWheelsSpeed, 0);
  executeMotorOrders();
  vehicleMoving = true;
  setDirectionIndicatorLights();
  if (reactionTimeOut != 0)
  {
    delay(reactionTimeOut);
  }
}

void sendMotorOrdersRightTurn (byte directionIndicator, byte reactionTimeOut, byte basisRPM)
{
  // basisRPM to be used later, now vehicle always drives at maxRPM
  float aux;
  aux = differentialTurnFraction * (float) maxRPM;
  byte rightWheelsSpeed  = (byte) aux;  // inner wheel
  byte leftWheelsSpeed  = maxRPM;// outer wheel
  transmitI2CLeftMotorMessageBlock (directionIndicator, reactionTimeOut, leftWheelsSpeed, 0);
  transmitI2CRightMotorMessageBlock (directionIndicator, reactionTimeOut, rightWheelsSpeed, 0);
  executeMotorOrders();
  vehicleMoving = true;
  setDirectionIndicatorLights();
  if (reactionTimeOut != 0)
  {
    delay(reactionTimeOut);
  }
}

void sendMotorOrdersStraight (byte directionIndicator, byte reactionTimeOut, byte vehicleSpeed, int distanceToTravel)
{
  byte rightWheelsSpeed  = vehicleSpeed;
  byte leftWheelsSpeed  = vehicleSpeed;

  mainMotorsOn = true;
  steeringMotorGoStraight(0);
  transmitI2CLeftMotorMessageBlock (directionIndicator, reactionTimeOut, leftWheelsSpeed, distanceToTravel);
  transmitI2CRightMotorMessageBlock (directionIndicator, reactionTimeOut, rightWheelsSpeed, distanceToTravel);
  executeMotorOrders();
  vehicleMoving = true;
  setDirectionIndicatorLights();
  if (reactionTimeOut != 0)
  {
    delay(reactionTimeOut);
  }
}

void sendMotorOrdersStop ()
{
  sendMotorOrdersStraight (0, 0, 0, 0);
  vehicleMoving = false;
  mainMotorsOn = false;
}

void transmitI2CLeftMotorMessageBlock(byte directionIndicator, byte reactionTimeOut, byte WheelsSpeed, int distanceToTravel)
{
  float pulsesNeeded = (float)distanceToTravel / distancePerPulse;
  _SERIAL_PRINT("transmitI2CLeftMotorMessageBlock, pulses needed:");
  _SERIAL_PRINTLN(pulsesNeeded);

  motionMotorOrder.direction = directionIndicator;     // 1 = forward, 2 = backward
  motionMotorOrder.duration = reactionTimeOut;
  motionMotorOrder.rpm = WheelsSpeed;              // speed in rotations per minute
  motionMotorOrder.pulsesNeeded = (int)pulsesNeeded;

  motionMotorOrder.slaveId = 11;
  motionMotorOrder.motorId = leftFront;                        // left side = 1,2 and 3. Right side = 4, 5 and 6
  transmitI2CMessage(miniProEEPROMChannel);

  motionMotorOrder.slaveId = 12;
  motionMotorOrder.motorId = leftMiddle;
  transmitI2CMessage(miniProEEPROMChannel);

  motionMotorOrder.slaveId = 13;
  motionMotorOrder.motorId = leftRear;
  transmitI2CMessage(miniProEEPROMChannel);

}
void transmitI2CRightMotorMessageBlock(byte directionIndicator, byte reactionTimeOut, byte WheelsSpeed, int distanceToTravel)
{
  float pulsesNeeded = (float)distanceToTravel / distancePerPulse;
  _SERIAL_PRINT("transmitI2CRightMotorMessageBlock, pulses needed:");
  _SERIAL_PRINTLN(pulsesNeeded);
  motionMotorOrder.direction = directionIndicator;     // 1 = forward, 2 = backward
  motionMotorOrder.duration = reactionTimeOut;
  motionMotorOrder.rpm = WheelsSpeed;              // speed in rotations per minute
  motionMotorOrder.pulsesNeeded = (int)pulsesNeeded;

  motionMotorOrder.slaveId = 11;
  motionMotorOrder.motorId = rightFront;
  transmitI2CMessage(miniProEEPROMChannel);

  motionMotorOrder.slaveId = 12;
  motionMotorOrder.motorId = rightMiddle;
  transmitI2CMessage(miniProEEPROMChannel);

  motionMotorOrder.slaveId = 13;
  motionMotorOrder.motorId = rightRear;
  transmitI2CMessage(miniProEEPROMChannel);

}

void addEquipmentOrderSegment (operationPossibilities equipmentOrderModePart, int EOPar1, int EOPar2, int routeListPosition)
{
  char logLineBuffer[80];
  int stepsize = 15;
  byte LSB, MSB;

  sprintf(logLineBuffer, "6, addEquipmentOrderSegment: %d, %d, %d, %d \0", equipmentOrderModePart, EOPar1, EOPar2, routeListPosition);
  writeLogLineOnFile(logLineBuffer);
  equipmentOrderType oneEquipmentOrder;
  oneEquipmentOrder.equipmentOrderMode = equipmentOrderModePart;
  oneEquipmentOrder.equipmentOrderModeParameter1 = EOPar1;
  oneEquipmentOrder.equipmentOrderModeParameter2 = EOPar2;
  //
  // calculate position and orientation from previous node and implement current order
  //
  if (equipmentOrders.size() == 0)
  {
    oneEquipmentOrder.positionAtCompletion.x = currentPosition.x;
    oneEquipmentOrder.positionAtCompletion.y = currentPosition.y;
    oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid = currentPosition.orientationRelativeToGrid;
  }
  else
  {
    equipmentOrderType aux = equipmentOrders.get(equipmentOrders.size());
    oneEquipmentOrder.positionAtCompletion.x = aux.positionAtCompletion.x;
    oneEquipmentOrder.positionAtCompletion.y = aux.positionAtCompletion.y;
    oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid = aux.positionAtCompletion.orientationRelativeToGrid;
  }

  switch (equipmentOrderModePart)
  {
    case (turnLeftMode):
      oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid = oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid + EOPar1;
      oneEquipmentOrder.positionAtCompletion.x = oneEquipmentOrder.positionAtCompletion.x + cos(oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid * 180.0 / PI) * EOPar2;
      oneEquipmentOrder.positionAtCompletion.y = oneEquipmentOrder.positionAtCompletion.y + sin(oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid * 180.0 / PI) * EOPar2;

      break;
    case (turnRightMode):
      oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid = oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid - EOPar1;
      oneEquipmentOrder.positionAtCompletion.x = oneEquipmentOrder.positionAtCompletion.x + cos(oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid * 180.0 / PI) * EOPar2;
      oneEquipmentOrder.positionAtCompletion.y = oneEquipmentOrder.positionAtCompletion.y + sin(oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid * 180.0 / PI) * EOPar2;
      break;
    case   (straightDrivingMode):
      oneEquipmentOrder.positionAtCompletion.x = oneEquipmentOrder.positionAtCompletion.x + cos(oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid * 180.0 / PI) * EOPar1;
      oneEquipmentOrder.positionAtCompletion.y = oneEquipmentOrder.positionAtCompletion.y + sin(oneEquipmentOrder.positionAtCompletion.orientationRelativeToGrid * 180.0 / PI) * EOPar1;
      break;
  }
  (routeListPosition >= 0) ? equipmentOrders.add(routeListPosition, oneEquipmentOrder) : equipmentOrders.add(oneEquipmentOrder);
}

void removeEquipmentOrder()
{
  boolean positionOK = true;
  boolean positionCheckRequired = false;
  equipmentOrderType myDeletedObject;
  myDeletedObject = equipmentOrders.pop();
  switch (myDeletedObject.equipmentOrderMode)
  {
    case (turnLeftMode):
      positionCheckRequired = false; // lets keep it false until we have a good running prototype
      break;
    case (turnRightMode):
      positionCheckRequired = false; // lets keep it false until we have a good running prototype
      break;
    case   (straightDrivingMode):
      positionCheckRequired = false;
      break;
  }
  //
  // positioncheck required if an order is executed where the ASC has moved.
  //
  if (positionCheckRequired)
  {
    if (orientationControl() != positionOK)
    {
      trimOrientation();
    }
    if (positionControl() != positionOK)
    {
      trimPosition();
    }
  }
}

equipmentOrderType getNextEquipmentOrder()
{
  return equipmentOrders.get(equipmentOrders.size() - 1);
}

void removeAllEquipmentOrders()
{
  while (equipmentOrders.size() > 0)
  {
    equipmentOrderType myDeletedObject;
    myDeletedObject = equipmentOrders.shift();
  }
}

operationPossibilities getNextActiveOperationalState()
{
  operationPossibilities aux = standByMode;

  if (operationsActiveMode)
  {

    if (!orderListDisplaySuspended)
    {
      displayEquipmentOrders();
    }
    if (equipmentOrders.size() > 0)
    {
      currentEquipmentOrder = getNextEquipmentOrder();
      aux = currentEquipmentOrder.equipmentOrderMode;
      debugSerialPrint(114, 114, false);//"Next operation: ");
      _SERIAL_PRINTLN(aux);
      switch (aux)
      {
        case (straightDrivingMode):
          orderedDistance = currentEquipmentOrder.equipmentOrderModeParameter1;
          break;
      }
      orderListDisplaySuspended = false;
    }
    else
    {
      orderListDisplaySuspended = true;
      operationsActiveMode = false;
    }
  }
  return aux;
}

bool positionControl()
{
  bool aux;
  aux = (abs(currentPosition.x - currentEquipmentOrder.positionAtCompletion.x) < marginOfErrorInGrid);
  if (aux) {
    aux = (abs(currentPosition.y - currentEquipmentOrder.positionAtCompletion.y) < marginOfErrorInGrid);
  }
  return aux;
}

bool angleCloseEnough(float angleReal, float angleNorm)
{
  float minAngle, maxAngle, checkAngle;

  checkAngle = angleReal;
  minAngle = angleNorm - angleMargin;
  maxAngle = angleNorm + angleMargin;
  if (minAngle <= 0)
  {
    minAngle = minAngle + 360;
    maxAngle = maxAngle + 360;
  }
  if (checkAngle < angleMargin)
  {
    checkAngle = checkAngle + 360;
  }
  bool aux = ((checkAngle >= minAngle) && (checkAngle <= maxAngle));
  return aux;
}


bool orientationControl()
{
  locateVehicle();
  bool aux = angleCloseEnough (currentPosition.orientationRelativeToGrid, currentEquipmentOrder.positionAtCompletion.orientationRelativeToGrid);
  return aux;
}

void trimOrientation()
{
  int correctionTurnAngle = currentPosition.orientationRelativeToGrid - currentEquipmentOrder.positionAtCompletion.orientationRelativeToGrid;
  if (correctionTurnAngle > 0)
  {
    leftTurn(correctionTurnAngle, 10);
  }
  else
  {
    rightTurn(abs(correctionTurnAngle), 10);
  }
}

void trimPosition()
{
  int correctionX = currentPosition.x - currentEquipmentOrder.positionAtCompletion.x;
  int correctionY = currentPosition.y - currentEquipmentOrder.positionAtCompletion.y;
  float correctionDistance = sqrt((correctionX * correctionX) + (correctionY * correctionY));
  float aux = (acos(correctionX / correctionDistance)) * 180.0 / PI;
  int correctionAngle = (int) aux + 360;
  int correctionTurnAngle = correctionAngle - currentPosition.orientationRelativeToGrid;
  if ((correctionAngle - 180.0) < currentPosition.orientationRelativeToGrid)
  {
    leftTurn(correctionTurnAngle, (int)correctionDistance);
  }
  else
  {
    rightTurn(correctionTurnAngle, (int)correctionDistance);
  }
}


void stopMainMotors()
{
  mainMotorsOn = false;
}

void displayWheelPulses (LiquidCrystal_I2C lcdDisplay, int passed, int cmToGo, bool horizontalLine)
{
  TCA9548A(mainI2CChannel);
  lcdDisplay.clear();
  lcdDisplay.setCursor(0, 0);
  lcdDisplay.print("pulses: ");
  lcdDisplay.setCursor(8, 0);
  lcdDisplay.print(passed);
  lcdDisplay.setCursor(0, 1);
  lcdDisplay.print((float)passed * distancePerPulse);
  lcdDisplay.setCursor(6, 1);
  lcdDisplay.print(" of ");
  lcdDisplay.setCursor(12, 1);
  lcdDisplay.print(cmToGo);
  lcdDisplay.setCursor(15, 0);
  (horizontalLine) ? lcdDisplay.print("+") : lcdDisplay.print("-");
}

void displayTurnProgress (LiquidCrystal_I2C lcdDisplay, float currentAngle, float previousAngle, float diffAngle, float countAngles, float target, bool horizontalLine)
{
  TCA9548A(mainI2CChannel);
  lcdDisplay.clear();
  lcdDisplay.setCursor(0, 0);
  lcdDisplay.print(currentAngle);
  lcdDisplay.setCursor(8, 0);
  lcdDisplay.print(previousAngle);
  lcdDisplay.setCursor(0, 1);
  lcdDisplay.print(diffAngle);
  lcdDisplay.setCursor(5, 1);
  lcdDisplay.print(countAngles);
  lcdDisplay.setCursor(10, 1);
  lcdDisplay.print(target);
  lcdDisplay.setCursor(15, 0);
  (horizontalLine) ? lcdDisplay.print("+") : lcdDisplay.print("-");
}

void displayHoistProgress (LiquidCrystal_I2C lcdDisplay, int height, bool displayLine)
{
  TCA9548A(mainI2CChannel);
  lcdDisplay.clear();
  lcdDisplay.setCursor(0, 1);
  lcdDisplay.print("Spreader at:");
  lcdDisplay.setCursor(12, 1);
  lcdDisplay.print(height);
  lcdDisplay.setCursor(15, 0);
  (displayLine) ? lcdDisplay.print("+") : lcdDisplay.print("-");
}

void displayTurnObjective (LiquidCrystal_I2C lcdDisplay, float currentAngle, float targetAngle)
{
  lcdDisplay.setCursor(12, 0);
  lcdDisplay.print(currentAngle);
  lcdDisplay.setCursor(12, 1);
  lcdDisplay.print(targetAngle);
}
void showMessage ( LiquidCrystal_I2C lcdDisplay, lcdParams lcdDisplayParams, int numberBlock, bool multipleLines = true)
{
  char textBlock[33];

  lcdDisplay.clear();
  for (int i = numberBlock; i < numberBlock + 2; i++)
  {
    lcdDisplay.setCursor(0, i - numberBlock);
    if ((i == numberBlock + 1))
    {
      if (multipleLines)
      {
        EETexts.readBlock (i, lcdDisplayParams.columns, false, textBlock);
        TCA9548A(mainI2CChannel);

        if (strlen(textBlock) > lcdDisplayParams.columns)
        {
          _SERIAL_PRINT (numberBlock);
          debugSerialPrint(275, 275, false);//":text to long: <bol>");
          _SERIAL_PRINT(textBlock);
          debugSerialPrint(276, 276, true);//"<eol> => truncated");
          textBlock[lcdDisplayParams.columns - 1] = 0;
        }
      }
      else
      {
        memset(textBlock, 32, 16);
        textBlock[16] = 0;
      }
    }
    else
    {
      EETexts.readBlock (i, lcdDisplayParams.columns, false, textBlock);
      TCA9548A(mainI2CChannel);
      if (strlen(textBlock) > lcdDisplayParams.columns)
      {
        _SERIAL_PRINT (numberBlock);
        debugSerialPrint(275, 275, false);//":text to long: <bol>");
        _SERIAL_PRINT(textBlock);
        debugSerialPrint(276, 276, true);//"<eol> => truncated");
        textBlock[lcdDisplayParams.columns - 1] = 0;
      }
    }
    lcdDisplay.print(textBlock);
  }
}

void debugSerialPrint(int blockNumberFirst, int blockNumberLast, boolean EOLine)
{
  int dummy;
  dummy = freeRam();
#ifdef DEBUG_PRINT
  serialPrintMaxLine(blockNumberFirst, blockNumberLast, 32, EOLine);
#endif
}

void showMessagePart (LiquidCrystal_I2C lcdDisplay, lcdParams lcdDisplayParams, char textBlock[], byte positionOnDisplay, byte lineOnDisplay)
{
  lcdDisplay.setCursor(positionOnDisplay, lineOnDisplay);
  if (strlen(textBlock) > (lcdDisplayParams.columns - positionOnDisplay))
  {
    debugSerialPrint(277, 278, false);//"ShowMessagePart contains a text that is too long: <bol>");
    _SERIAL_PRINT(textBlock);
    debugSerialPrint(276, 276, true);//"<eol> => truncated");
    textBlock[lcdDisplayParams.columns - 1] = 0;

  }
  textBlock[lcdDisplayParams.columns - positionOnDisplay - 1] = 0;
  lcdDisplay.print(textBlock);

}

void serialPrintMaxLine(int blockNumberFirst, int blockNumberLast, byte blockSize, boolean EOLine)
{
  char textBlock[33];

  for (int i = blockNumberFirst; i <= blockNumberLast; i++)
  {
    EETexts.readBlock (i, blockSize, true, textBlock);// "Step 1:
    TCA9548A(mainI2CChannel);
    _SERIAL_PRINT(textBlock);
    _SERIAL_PRINT(" ");

  }

  if (EOLine)
  {
    _SERIAL_PRINTLN("");
  }

}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  int fr = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  if (fr < minimumFreeRam)
  {
    minimumFreeRam = fr;
    if (fr < 1024)
    {
      _SERIAL_PRINTLN ("ALARM -- MEMORY BELOW 1K BYTES -- ALARM");
    }
  }
  return fr;
}

void setCommunicationLED(byte Pin, bool high, int minimumDelayAfterSetting)
{
  if (high)
  {
    digitalWrite(Pin, HIGH) ;
    if (Pin != callFromInterruptPin)
    {
      checkI2CReturn = true;
    }
  }
  else
  {
    digitalWrite(Pin, LOW);
    if (Pin != callFromInterruptPin)
    {
      checkI2CReturn = false;
      timeOutI2CCount = 0;
    }
  }
  if (minimumDelayAfterSetting > 0)
  {
    waitFor(minimumDelayAfterSetting);
  }
}

void setPinModes()
{
  pinMode(masterReadyForSlavePin,     OUTPUT);
  pinMode(MISO,                       OUTPUT);
  pinMode(callFromInterruptPin,       OUTPUT);
  pinMode(callToI2CPin,               OUTPUT);
  pinMode(waitForAnswerPin,           OUTPUT);
  pinMode(steeringMotorPowerSwitch,   OUTPUT);
  pinMode(radioActiveLightPin,        OUTPUT);
  pinMode(startupCloseDownSlave,      OUTPUT);
  pinMode(connectI2CSlave,            OUTPUT);
  pinMode(startupCloseDownMotors,     OUTPUT);
  pinMode(connectI2CMotors,           OUTPUT);
  pinMode(motorSpreaderEnablePin,     OUTPUT);
  pinMode(motorSpreaderPin1,          OUTPUT);
  pinMode(motorSpreaderPin2,          OUTPUT);
  pinMode(buzzerPin,                  OUTPUT);
  pinMode(lightSourcePin,             OUTPUT);
  pinMode(spreaderLockPin,            OUTPUT);
  pinMode(poleMotorPin1,              OUTPUT);
  pinMode(poleMotorPin2,              OUTPUT);
  pinMode(poleMotorPin3,              OUTPUT);
  pinMode(poleMotorPin4,              OUTPUT);
  pinMode(compassMotorPin1,           OUTPUT);
  pinMode(compassMotorPin2,           OUTPUT);
  pinMode(compassMotorPin3,           OUTPUT);
  pinMode(compassMotorPin4,           OUTPUT);
  pinMode(forwardIndicatorPin2,       OUTPUT);
  pinMode(backwardIndicatorPin2,      OUTPUT);
  pinMode(laserPointerPin,            OUTPUT);
  pinMode(spreaderLightsPin,          OUTPUT);
  pinMode(wakeUpPin,  OUTPUT);

  pinMode(lightSensorPin,             INPUT);
  pinMode(leftFollowLineSensorPin,    INPUT);
  pinMode(rightFollowLineSensorPin,   INPUT);
  pinMode(interruptStopMainMotorsPin, INPUT);
  pinMode(interruptFromSlavePin,      INPUT);

}
void setup()
{
  byte rawContent[maxMessageParameters];
  int radioStartTries = 0;

  setPinModes();

  Serial.begin(74880);

  char logLineBuffer[80];
  char lineToDisplay[33];
  byte wireError;
  long startResetMoment;
  bool slaveStarted;
  byte devicesReady;

  _SERIAL_PRINTLN("===============================");
  _SERIAL_PRINTLN("");
  _SERIAL_PRINTLN("Main module of Straddle started");
  runNumber = EEPROM.read(4095) << 8 | EEPROM.read(4096);
  runNumber = runNumber + 1;
  EEPROM.write(4095, (runNumber >> 8));
  EEPROM.write(4096, (runNumber & 0xFF));
  _SERIAL_PRINTLN("");
  _SERIAL_PRINTLN("===============================");
  digitalWrite(masterReadyForSlavePin, LOW);

  lcdMasterParams.columns  = 16;
  lcdMasterParams.rows     =  2;
  lcdSlaveParams.columns   = 16;
  lcdSlaveParams.rows      =  2;
  lcdMotorsParams.columns  = 20;
  lcdMotorsParams.rows     =  4;

  routeListNodeNumber = 0;

  digitalWrite(radioActiveLightPin, LOW);

  Wire.begin();
  Wire.setClock(400000);
  _SERIAL_PRINT("Started runnumber: ");
  _SERIAL_PRINTLN(runNumber);
  TCA9548A(mainI2CChannel);
  _SERIAL_PRINT("Na TCA9548A - ");
  //Wire.setWireTimeout(3000, true); //timeout value in uSec - SBWire uses 100 uSec, so 1000 should be OK
  //
  // power-off slave
  //
  digitalWrite(startupCloseDownSlave, HIGH); // power off slave
  digitalWrite(connectI2CSlave, HIGH);
  //
  // power-off motor mcs
  //
  digitalWrite(startupCloseDownMotors, HIGH); // power off motors (Arduino Pro Mini)
  digitalWrite(connectI2CMotors, HIGH);
  //
  // power-off steering motors
  //
  lcdMotors.init();
  delay(1000);
  _SERIAL_PRINTLN("Na lcdMotors.init");
  lcdMotors.backlight();
  lcdMotors.clear();

  EETexts.begin();
  delay(250);
  _SERIAL_PRINTLN("Na EETexts.begin");

  EETexts.readBlock (111, 16, true, lineToDisplay);// "Motor display
  _SERIAL_PRINTLN("Na EETexts.readBlock");

  showMessagePart (lcdMotors, lcdMotorsParams, lineToDisplay, 0, 0);
  lcdMaster.init();
  lcdMaster.backlight();
  lcdMaster.clear();
  EETexts.readBlock (113, 16, true, lineToDisplay);// "Master display
  showMessagePart (lcdMaster, lcdMasterParams, lineToDisplay, 0, 0);
  lcdMaster.setCursor(12, 0);
  lcdMaster.print(runNumber);

  lcdSlave.init();
  lcdSlave.backlight();
  lcdSlave.clear();
  EETexts.readBlock (112, 16, true, lineToDisplay);// "Slave display
  showMessagePart (lcdSlave, lcdSlaveParams, lineToDisplay, 0, 0);
  lcdSlave.setCursor(12, 0);
  lcdSlave.print(runNumber);

  digitalWrite(wakeUpPin, HIGH);
  SPCR |= _BV(SPE);  // turn on SPI in slave mode
  SPCR |= _BV(SPIE); // turn on interrupts
  SPIBufferPosition = 0;
  hasData = false;

  lcdSlave.clear();
  lcdSlave.print("Wait for radio");
  while (!radioIsWorking)
  {
    if (!startingUp)
    {
      radioStartTries++;
      radioIsWorking = getRadioStatus();
      lcdSlave.setCursor(0, 1);
      if (radioIsWorking)
      {
        lcdSlave.print("Radio is operational");
        digitalWrite(radioActiveLightPin, HIGH);
        _SERIAL_PRINTLN("Radio on");
      }
      else
      {
        lcdSlave.print("Radio off : ");
        lcdSlave.print(radioStartTries);
        digitalWrite(radioActiveLightPin, LOW);
        _SERIAL_PRINTLN("Radio off");
        waitFor(20000);
      }
    }
    else
    {
      startingUp = (millis() < 20000);
      waitFor(1000);
    }
  }

  TCA9548A(mainI2CChannel);
  DateTime now = defineNow.now();
  sprintf(logLineBuffer, "%02i-%02i-%02i ASC Run number : %i \0", now.day(), now.month(), now.year(), runNumber);
  writeLogLineOnFile(logLineBuffer);

  TCA9548A(miniProEEPROMChannel);
  if (toFSensor.init())
  {

    digitalWrite(laserPointerPin, HIGH);

    toFSensor.setDistanceMode(VL53L1X::Long);
    toFSensor.setMeasurementTimingBudget(50000);
    toFSensor.setTimeout(5000);
    //toFSensor.startContinuous(50);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.

    TCA9548A(miniProEEPROMChannel);
    bool measurementReliable = reliableDistance();
    debugSerialPrint(122, 122, false);//"Test measurement: ";
    _SERIAL_PRINTLN(distanceInMM);
    digitalWrite(laserPointerPin, HIGH);

    debugSerialPrint(123, 124, true); //"Step 10: toFSensor initialisation enabled";
    showMessage(lcdMaster, lcdMasterParams, 21); // "Step 10         ","toFSensor ok    ";
    tofSensorStatus = operational;
  }
  else
  {
    debugSerialPrint(125, 125, true); //"Step 10: toFSensor ERROR";
    showMessage(lcdMaster, lcdMasterParams, 23); //"Step 10         ","toFSensor ERROR ";
    tofSensorStatus = failure;
    soundAlarm(1, 1000);
    delay(2000);
  }

  Timer1.initialize(); // default is 1 second
  Timer1.attachInterrupt(checkTimeOuts);
  devicesReady = scanOnePass();
  debugSerialPrint(3, 4, true); //Step 1: init done
  setCommunicationLED (callFromInterruptPin, LEDOn, 2000);
  setCommunicationLED (callFromInterruptPin, LEDOff, 0);
  setCommunicationLED (callToI2CPin, LEDOn, 2000);
  setCommunicationLED (callToI2CPin, LEDOff, 0);
  setCommunicationLED (waitForAnswerPin, LEDOn, 2000);
  setCommunicationLED (waitForAnswerPin, LEDOff, 0);
  debugSerialPrint(31, 32, true); //Step 14: init steering motor start

  digitalWrite(steeringMotorPowerSwitch, HIGH);
  TCA9548A(miniProEEPROMChannel);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  showMessage (lcdMaster, lcdMasterParams, 31);

  for (int i = 0; i < 3; i++)
  {
    setServoToAngle (leftFront, 60);
    setServoToAngle (leftRear, 60);
    setServoToAngle (rightFront, 60);
    setServoToAngle (rightRear, 60);
    delay(1000);
    setServoToAngle (leftFront, 120);
    setServoToAngle (leftRear, 120);
    setServoToAngle (rightFront, 120);
    setServoToAngle (rightRear, 120);
    delay(1000);
  }
  delay(3000);

  setServoToAngle (leftFront, 90);
  delay(500);
  setServoToAngle (leftFront, 0);
  delay(500);
  setServoToAngle (leftRear, 90);
  delay(500);
  setServoToAngle (leftRear, 0);
  delay(500);
  setServoToAngle (rightFront, 90);
  delay(500);
  setServoToAngle (rightFront, 0);
  delay(500);
  setServoToAngle (rightRear, 90);
  delay(500);
  setServoToAngle (rightRear, 0);
  delay(500);
  setServoToAngle (leftFront, 90);
  setServoToAngle (leftRear, 90);
  setServoToAngle (rightFront, 90);
  setServoToAngle (rightRear, 90);
  delay(2000);
  digitalWrite(steeringMotorPowerSwitch, LOW);
  waitFor(250);
  masterReceive = 0;
  showMessage(lcdMaster, lcdMasterParams, 7);
  debugSerialPrint(7, 8, true); //Step 3 EEPROM
  //
  devicesReady = scanOnePass();
  //
  // set output pins
  //

  debugSerialPrint(106, 108, true); //"106-108Step 6: Initialized output pins and interrupts (collision prevent, spreader motor, IR receive", true);
  showMessage(lcdMaster, lcdMasterParams, 13); // "Step 6          ","Init o-pins ok  ");
  //
  // inititialising spreader position and motor
  //
  spreaderCurrentPosition = spreaderPositionHigh;
  //
  // initialising matrix board for navigation lights
  //
  debugSerialPrint(118, 118, true); //"Step 7 : Matrix board      set";
  showMessage(lcdMaster, lcdMasterParams, 15); //"Step 7  Matrix  ","board init      ";
  testNavigationLights();
  setDirectionIndicatorLights();
  setDrivingDirection(drivingBackwards);
  delay(3000);
  setDirectionIndicatorLights();
  setDrivingDirection(drivingForwards);
  setDirectionIndicatorLights();
  showMessage(lcdMaster, lcdMasterParams, 17); // "Step 8      set ","direction lights";
  debugSerialPrint(119, 119, true); //"Step 8 : driving direction";
  currentHeading = gridOrientationRelativeToNorth;
  initCompass();
  initialHeading = getHeading();

  orientationOfGridRelativeToNorth = initialHeading ; // in studeerkamer op de opera Zwijndrecht
  orientationVehicleRelativeToNorth = initialHeading; // SC gepositioneerd in de positieve richting op de x-as
  debugSerialPrint(120, 120, true); //"Step 9: Initial heading determined";
  debugSerialPrint(121, 121, false);//("--Initial Heading: ";
  _SERIAL_PRINTLN(initialHeading);
  orderedHeading = initialHeading;
  currentHeading = initialHeading;
  digitalWrite(laserPointerPin, HIGH);

  digitalWrite(laserPointerPin, LOW);
  debugSerialPrint(126, 126, true); //"Step 11: toF - A-SC aligned    ";
  showMessage(lcdMaster, lcdMasterParams, 25); //"Step 11         ","aligned toF-A-SC";
  spreaderLightsOn();
  debugSerialPrint(127, 127, true); //"Step 12: setup spreader lights";
  showMessage(lcdMaster, lcdMasterParams, 27); //"Step 12    init ","spreader lights";

  long startBlinkTime = millis();
  while (millis() <= (startBlinkTime + 6500))
  {  };
  spreaderLightsOff();

  // Interrupt parameters
  attachInterrupt(digitalPinToInterrupt(interruptFromSlavePin), triggerFromSlave,  RISING);
  attachInterrupt(digitalPinToInterrupt(interruptSpreaderFinishPin), triggerFromSpreader,  RISING);

  debugSerialPrint(128, 128, true); //"Step 13: init guidance system");
  showMessage(lcdMaster, lcdMasterParams, 29); //"Step 13    init ","guidance system ");
  debugSerialPrint(129, 129, false);//("activeOperationalState at start = ");
  _SERIAL_PRINT(activeOperationalState);
  debugSerialPrint(130, 130, true); //" (0 = wait, xx = object detected)");
  debugSerialPrint(131, 131, true); //"Step 14: init steering motor start");
  showMessage(lcdMaster, lcdMasterParams, 31);// "Step 14    init ","steering motors ");

  slaveStarted = false;
  while (!slaveStarted)
  {
    digitalWrite(startupCloseDownSlave, LOW); // power on slave
    digitalWrite(connectI2CSlave, LOW);

    //
    waitFor(1000);
    startResetMoment = millis();

    showMessage(lcdMaster, lcdMasterParams, 33); // "Step 15 init    ","wait until ready");
    resetSlave();
    slaveInitialising = true;
    debugSerialPrint(132, 132, true); //"Step 15: Wait until slave has started");


    while (slaveInitialising && ((millis() - startResetMoment) < 60000))
    {
      checkSlaveData();
      waitFor(100);
    }
    slaveStarted = !slaveInitialising;

  }
  digitalWrite(masterReadyForSlavePin, HIGH);

  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(mainI2CChannel);
  Wire.beginTransmission(slaveNodeAddress);
  Wire.write(informSlaveToStart);
  Wire.endTransmission();
  setCommunicationLED(callToI2CPin, LEDOff, 0);

  // Initialising radio code
  showMessage(lcdMaster, lcdMasterParams, 35); //"Step 18 Comm    ","Wait for comm   ");
  delay(1000);
  radioIsWorking = getRadioStatus();
  if (radioIsWorking)
  {
    showMessage(lcdMaster, lcdMasterParams, 39);    //"Step 19         ","Radio com    ok ");
    digitalWrite(radioActiveLightPin, HIGH);
  }
  else
  {
    showMessage(lcdMaster, lcdMasterParams, 40);    //"Step 19         ","Radio com   NOK ");
    digitalWrite(radioActiveLightPin, LOW);
  }

  //debugSerialPrint(140, 140, true); //"Step 20: determine start position");
  locateVehicle();
  showMessage(lcdMaster, lcdMasterParams, 41);    //"Step 20         ","loc. ASC on grid");
  debugSerialPrint(141, 141, false);//("--17Initial position: (");
  logPosition();

  digitalWrite(startupCloseDownMotors, LOW); // power on motors (Arduino Pro Mini)
  _SERIAL_PRINTLN("Before connecting I2C motors");
  long auxInt = waiting();
  digitalWrite(connectI2CMotors, LOW);
  digitalWrite(steeringMotorPowerSwitch, HIGH);
  delay(250);

  attachInterrupt(digitalPinToInterrupt(interruptStopMainMotorsPin), stopMainMotors, RISING);
  _SERIAL_PRINTLN("Start scanning....");
  showMessage(lcdMaster, lcdMasterParams, 43, false); //"Step 16 I2C     ","Scan for devices");
  debugSerialPrint(142, 142, true); //"Step 16: Scan for all I2C devices");
  auxInt = waiting();
  scanDevicesUntilFound();
  auxInt = waiting();
  debugSerialPrint(143, 143, true); //"Step 17: all I2C devices are found");
  lastMotorDisplayTime = 0;
  waitFor(2000);
  debugSerialPrint(144, 144, true); //"Step 21: test motors");

  showMessage(lcdMaster, lcdMasterParams, 47);     //"Step 21    init ","movement motors");*/
}

long waiting()
{
  long aux = 0;
#ifdef TEST_MODE
  _SERIAL_PRINT("waiting for input..");
  while (!Serial.available());
  aux = Serial.parseInt();
  _SERIAL_PRINTLN(aux);
#endif

  return aux;

}
byte scanOnePass()
{
  char textBlock[33];
  int nDevices = 0;
  int scanCount = 0;
  byte I2C7 = mainI2CChannel;
  byte I2C1 = miniProEEPROMChannel;
  byte deviceOnI2CBus[17]       = {I2C7,  I2C1,  I2C1,  I2C1,  I2C1,  I2C7,  I2C7,  I2C7,  I2C7,  I2C7,  I2C1,  I2C1,  I2C1,  I2C7,  I2C7,  I2C7,  I2C7};
  byte requiredDevices[17]      = {0x09,  0x0B,  0x0C,  0x0D,  0x0E,  0x19,  0x1E,  0x23,  0x25,  0x27,  0x29,  0x40,  0x50,  0x57,  0x68,  0x69,  0x70};
  bool requiredDevicedFound[17] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
  //
  //  I2C addresses are:
  //  22:27:36.168 -> I2C device found at address 0x09  ! Arduino Mega - slave - general purpose  (I2C7)
  //  22:27:36.201 -> I2C device found at address 0x0B  ! Arduino Pro Mini slave - front  wheels 1 and 4 (I2C1)
  //  22:27:36.235 -> I2C device found at address 0x0C  ! Arduino Pro Mini slave - middle wheels 2 and 5 (I2C1)
  //  22:27:36.269 -> I2C device found at address 0x0D  ! Arduino Pro Mini slave - back   wheels 3 and 6 (I2C1)
  //                  I2C device found at address 0x0E  ! Arduino Nano display (I2C1)
  //                  I2C device found at address 0x19  ! LSM303 Accelerometer (I2C7)
  //  22:27:36.337 -> I2C device found at address 0x1E  ! LSM303 Magnetometer (I2C7)
  //  22:27:36.337 -> I2C device found at address 0x23  ! LCD 16x2 - Arduino master computer (I2C7)
  //                  I2C device found at address 0x25  ! LCD 20x4 - Arduino master computer (I2C7)
  //  22:27:36.337 -> I2C device found at address 0x27  ! LCD 16x2 - Arduino slave computer (I2C7)
  //  22:27:36.404 -> I2C device found at address 0x29  ! VL53L1X  - Time Of Flight sensor for positioning (I2C1)
  //  22:27:36.404 -> I2C device found at address 0x40  ! PCA9685  - Servo driver board  (I2C1)
  //  22:27:36.404 -> I2C device found at address 0x50  ! EEPROM containing all texts (I2C1)
  //  22:27:36.438 -> I2C device found at address 0x57  ! Unknown RTC-module additional address (I2C7)
  //  22:27:36.473 -> I2C device found at address 0x68  ! Real time clock (I2C7)
  //                  I2C device found at address 0x69  ! LSM303 Gyroscope (I2C7)
  //                  I2C device found at address 0x70  ! I2C bus multiplexer (I2C7)
  //   other  communication via SPI between master and  micro SD card
  //   and
  //   via UART between master and Wemos Mini Pro dor wireless communication module - cimmunication with ground station
  //
  TCA9548A(miniProEEPROMChannel);
  lcdMotors.init();
  nDevices = 0;
  for (byte deviceAddress = 0; deviceAddress < sizeof(requiredDevices); ++deviceAddress)
  {
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    TCA9548A(deviceOnI2CBus[deviceAddress]);
    Wire.beginTransmission(requiredDevices[deviceAddress]);
    byte error = Wire.endTransmission();
    _SERIAL_PRINT("Address:")
    _SERIAL_PRINT_HEX(requiredDevices[deviceAddress])
    _SERIAL_PRINT(", I2CChannel:")
    _SERIAL_PRINT(deviceOnI2CBus[deviceAddress])
    _SERIAL_PRINT(", error status: ")
    _SERIAL_PRINTLN(error);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    if (error == 0)
    {
      requiredDevicedFound[deviceAddress] = true;
      nDevices++;
    }
    else
    {
      requiredDevicedFound[deviceAddress] = false;
    }
  }
  TCA9548A(deviceOnI2CBus[6]); // switch back to I2C bus of the lcdMaster (0x23) to display result
  for (byte deviceAddress = 0; deviceAddress < sizeof(requiredDevices); ++deviceAddress)
  {
    lcdMotors.setCursor(deviceAddress, 1);
    (requiredDevicedFound[deviceAddress]) ? lcdMotors.print('x') : lcdMotors.print('0');
  }
  return nDevices;
}

void scanDevicesUntilFound()
{
  char logLineBuffer[80];
  char textBlock[33];
  int nDevices = 0;
  int scanCount = 0;
  byte requiredDevices = 16;
  //
  //    See scanOnePass
  //
  debugSerialPrint(150, 150, true); //" Scanning...Until FOund"

  while (nDevices < requiredDevices) // minus 1 because of the separate I2C bus for EEPROM
  {
    ++scanCount;
    lcdMotors.init();
    lcdMotors.setCursor(0, 0);
    lcdMotors.print("I2C test ");
    lcdMotors.setCursor(12, 0);
    lcdMotors.print(scanCount);
    nDevices = scanOnePass();
    waitFor(3000); // Wait 3 seconds for next scan
  }
}

void logPosition()
{
  char textBlock[80];
  char logLineBufferLocal[80];
  char str1[10];
  char str2[10];
  char str3[10];
  dtostrf(currentPosition.x, 5, 1, str1);
  dtostrf(currentPosition.y, 5, 1, str2);
  dtostrf(currentPosition.orientationRelativeToGrid, 5, 0, str3);
  EETexts.readBlock (152, 32, true, textBlock);    //11,P(%s, %s; %s)
  sprintf(logLineBufferLocal, textBlock, str1, str2, str3);
  writeLogLineOnFile(logLineBufferLocal);
  lcdSlave.clear();
  lcdSlave.setCursor(0, 0);
  lcdSlave.print("(");
  lcdSlave.setCursor(1, 0);
  lcdSlave.print(str1);
  lcdSlave.setCursor(8, 0);
  lcdSlave.print(",");
  lcdSlave.setCursor(9, 0);
  lcdSlave.print(str2);
  lcdSlave.setCursor(14, 0);
  lcdSlave.print(")");
  lcdSlave.setCursor(0, 1);
  lcdSlave.print("heading");
  lcdSlave.setCursor(8, 1);
  lcdSlave.print(str3);
  lcdSlave.setCursor(13, 1);
  lcdSlave.print("D");
}

bool getRadioStatus()
{
  bool aux = getRadioStatusFromSerial();
  return aux;
}

bool getRadioStatusFromSerial()
{
  bool aux;
  byte rawContent[maxMessageParameters];

  memset (rawContent, 0, sizeof(rawContent));
  rawContent[0] = (byte)checkRadioStatus;

  mS_sendMessage = prepareSerialMessage (straddleName + "M", straddleName + "R", straddleName + "R", radioStatusRequest, 1, &rawContent[0]);
  mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
  displayMessage(&mR_receiveMessage, false);
  displayMessageLCD(mS_sendMessage, sent, 0);
  aux = ((mR_receiveMessage.messageTypeId == radioStatusReply) && (mR_receiveMessage.content[0] == radioOn));

  return aux;
}

void setDirectionIndicatorLights()
{
  if (drivingDirection == drivingForwards)
  {
    digitalWrite(forwardIndicatorPin2, HIGH);
    digitalWrite(backwardIndicatorPin2, LOW);
  }
  else
  {
    digitalWrite(forwardIndicatorPin2, LOW);
    digitalWrite(backwardIndicatorPin2, HIGH);
  }

  sendMatrixBoardMessage(determineMatrixBoardPictogram(spreaderCurrentPosition, spreaderOccupied, drivingDirection, executionStatus));
}

void sendMatrixBoardMessage (byte matrixBoardIcon)
{
  TCA9548A(miniProEEPROMChannel);
  setCommunicationLED(callToI2CPin, LEDOn, 5);
  setCommunicationLED(callToI2CPin, LEDOff, 0);
  Wire.beginTransmission (DASHBOARD_ADDRESS);
  for (int i = 0; i < 9; i++)
  {
    Wire.write(s[matrixBoardIcon][i]);
  }
  Wire.endTransmission();
  delay(50);
}

void testNavigationLights()
{
  lcdMaster.clear();
  lcdMaster.setCursor(10, 0);
  lcdMaster.print(0);
  sendMatrixBoardMessage(suls);
  waitFor(100);
  sendMatrixBoardMessage(sulf);
  waitFor(100);
  sendMatrixBoardMessage(sulb);
  waitFor(100);
  sendMatrixBoardMessage(sulr);
  lcdMaster.setCursor(10, 0);
  lcdMaster.print(1);
  waitFor(100);
  sendMatrixBoardMessage(sull);
  waitFor(100);
  sendMatrixBoardMessage(suus);
  waitFor(100);
  sendMatrixBoardMessage(suuf);
  waitFor(100);
  sendMatrixBoardMessage(suub);
  lcdMaster.setCursor(10, 0);
  lcdMaster.print(2);
  waitFor(100);
  sendMatrixBoardMessage(suur);
  waitFor(100);
  sendMatrixBoardMessage(suul);
  waitFor(100);
  sendMatrixBoardMessage(sdls);
  waitFor(100);
  sendMatrixBoardMessage(sdlf);
  lcdMaster.setCursor(10, 0);
  lcdMaster.print(3);
  waitFor(100);
  sendMatrixBoardMessage(sdlr);
  waitFor(100);
  sendMatrixBoardMessage(sdlb);
  waitFor(100);
  sendMatrixBoardMessage(sdll);
  waitFor(100);
  sendMatrixBoardMessage(sdub);
  waitFor(100);
  lcdMaster.setCursor(10, 0);
  lcdMaster.print(4);
  sendMatrixBoardMessage(sdus);
  waitFor(100);
  sendMatrixBoardMessage(sduf);
  waitFor(100);
  sendMatrixBoardMessage(sdur);
  waitFor(100);
  sendMatrixBoardMessage(sdul);
  waitFor(100);
  sendMatrixBoardMessage(suls);
}


byte determineMatrixBoardPictogram (int spreaderPosition, boolean spreaderLoadStatus, byte moveDirection, int execution)
{
  switch (spreaderPosition)
  {
    case spreaderPositionHigh:
      if (spreaderLoadStatus)
      {
        switch (execution)
        {
          case 3:
            return sull;
            break;
          case 2:
            return sulr;
            break;
          case 0:
            return suls;
            break;
          case 1:
            if (moveDirection == drivingForwards)
            {
              return sulf;
            }
            else
            {
              return sulb;
            }
            break;
        }
      }
      else
      {
        switch (execution)
        {
          case 3:
            return suul;
            break;
          case 2:
            return suur;
            break;
          case 0:
            return suus;
            break;
          case 1:
            if (moveDirection == drivingForwards)
            {
              return suuf;
            }
            else
            {
              return suub;
            }
            break;
        }
      }
      break;
    case spreaderPositionLow:
      if (spreaderLoadStatus)
      {
        switch (execution)
        {
          case 3:
            return sdll;
            break;
          case 2:
            return sdlr;
            break;
          case 0:
            return sdls;
            break;
          case 1:
            if (moveDirection == drivingForwards)
            {
              return sdlf;
            }
            else
            {
              return sdlb;
            }
            break;
        }
      }
      else
      {
        switch (execution)
        {
          case 3:
            return sdul;
            break;
          case 2:
            return sdur;
            break;
          case 0:
            return sdus;
            break;
          case 1:
            if (moveDirection == drivingForwards)
            {
              return sduf;
            }
            else
            {
              return sdub;
            }
            break;
        }
      }
      break;
  }
}

void spreaderLightsOn()
{
  digitalWrite(spreaderLightsPin, HIGH);
  spreaderLightIsOn = true;
  checkSpreaderLights = true;
}

void spreaderLightsOff()
{
  digitalWrite(spreaderLightsPin, LOW);
  spreaderLightIsOn = false;
  checkSpreaderLights = false;

}

void checkTimeOuts()
{
  if (checkSpreaderHeight)
  {
    askForSpreaderHeight = true;
  }
  if (checkWheelPulseTimeOut)
  {
    askForPulsesPassed = true;
  }
  if (checkSpreaderLights)
  {
    spreaderLightsBlinking();
  }

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
    _SERIAL_PRINT(", time out? ");
    _SERIAL_PRINTLN(messageTimedOut);
  }

  if (checkI2CReturn)
  {
    switch (timeOutI2CCount)
    {
      case 5:
        //digitalWrite(startupCloseDownSlave, HIGH); // power off slave
        timeOutI2CCount++;
        break;
      case 6:
        //digitalWrite(startupCloseDownSlave, LOW); // power on slave
        timeOutI2CCount = 0;
        break;
      default:
        timeOutI2CCount++;
        break;
    }
  }
}

void spreaderLightsBlinking()
{
  if (spreaderLightIsOn)
  {
    digitalWrite(spreaderLightsPin, LOW);
    spreaderLightIsOn = false;
  }
  else
  {
    digitalWrite(spreaderLightsPin, HIGH);
    spreaderLightIsOn = true;
  }
}

void followBlackLinesUntilStop()
{
  int leftSensorState;
  int rightSensorState;
  bool continueMoving = true;
  byte previousCombination = 0;
  bool leftSeesLine = false;
  bool rightSeesLine = false;
  bool previousLeftValue = false;
  bool previousRightValue = false;
  bool arrived = false;

  _SERIAL_PRINTLN("in followBlackLinesUntilStop");
  while (continueMoving)
  {
    leftSeesLine  = !digitalRead(leftFollowLineSensorPin);
    rightSeesLine = !digitalRead(rightFollowLineSensorPin);
#ifdef DEBUG_PRINT
    (leftSeesLine) ? Serial.print("fbl:sensor left: TRUE") : Serial.print("fbl:sensor left: FALSE");
    (rightSeesLine) ? Serial.print(", sensor right: TRUE -> ") : Serial.println(", sensor right: FALSE -> ");
#endif
    if (!((previousLeftValue == leftSeesLine) && (previousRightValue == rightSeesLine)))
    {
      switch (leftSeesLine)
      {
        case true:
          switch (rightSeesLine)
          {
            case true:
              _SERIAL_PRINTLN("Stop at black line");
              sendMotorOrdersStop();
              continueMoving = false;
              break;
            case false:
              _SERIAL_PRINTLN("Turn right");
              arrived = guidedDriveAdjustToTheRight();
              break;
          }
          break;
        case false:
          switch (rightSeesLine)
          {
            case true:
              _SERIAL_PRINTLN("Turn left");
              arrived = guidedDriveAdjustToTheLeft();
              break;
            case false:
              _SERIAL_PRINTLN("Go straight");
              steeringMotorGoStraight(0);
              break;
          }
          break;
      }
      delay(100);
      previousLeftValue = leftSeesLine;
      previousRightValue = rightSeesLine;
    }
    if (arrived)
    {
      continueMoving = false;
    }
  }
}

float getHeading()
{
  double aux;

  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(mainI2CChannel);
  delay(40);
  compass.read();
  aux = compass.heading();
  if (isnan(aux))
  {
    _SERIAL_PRINT(" (!!! NO VALID MEASUREMENT !!!) ");
    aux = currentHeading + 1;
  }
  setCommunicationLED(callToI2CPin, LEDOff, 0);

  return aux;
}

void displayEquipmentOrders()
{
  char logLineBufferLocal[80];
  char textBlock[41];

  equipmentOrderType order;

  debugSerialPrint(183, 183, false);//Nr of orders in the list:
  _SERIAL_PRINTLN(equipmentOrders.size());

  for (int i = 0; i < equipmentOrders.size(); ++i) {

    // Get equipment order from the list
    order = equipmentOrders.get(i);

    memset (logLineBufferLocal, 0, sizeof(logLineBufferLocal));
    EETexts.readBlock (184, 32, true, textBlock); //
    sprintf(logLineBufferLocal, textBlock, i, order.equipmentOrderMode, order.equipmentOrderModeParameter1);
    writeLogLineOnFile(logLineBufferLocal);
    _SERIAL_PRINT(logLineBufferLocal);
    _SERIAL_PRINT(", ");
    _SERIAL_PRINTLN(order.equipmentOrderModeParameter2);
  }
}

void displayMessageLCD (aMessage __Message, messageDirection __indicator, int delaySeconds)
{
  char ind;
  delaySeconds = delaySeconds * delayFactor;
  if (delaySeconds > 0)
  {
    (__indicator == received) ? ind = '<' : ind = '>';

    lcdMaster.clear();
    lcdMaster.setCursor(0, 0);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.totalMessageLength);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.totalContentItems);
    lcdMaster.print(ind);
    lcdMaster.print((__Message.messageNumberHighByte << 8) | __Message.messageNumberLowByte);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.messageTypeId);
    lcdMaster.setCursor(0, 1);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.content[0]);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.content[1]);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.content[2]);
    lcdMaster.print(ind);
    lcdMaster.print(__Message.content[3]);

    for (int i = 1; i <= delaySeconds; i++)
    {
      lcdMaster.setCursor(14, 1);
      lcdMaster.print(" ");
      lcdMaster.setCursor(14, 1);
      lcdMaster.print(i);
      delay(1000);
    }
    lcdMaster.clear();
  }
}

void decodeSerialMessage()
{
  messageTypeIds aux = processMessageFromSerial();
}

messageTypeIds processMessageFromSerial()
{
  byte messageTypeId;
  byte rawContent[6];
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
        mS_sendMessage = prepareSerialMessage (straddleName + "M", straddleName + "R", straddleName + "R", messageStatusReply, 1, &rawContent[0]);
        displayMessage(&mS_sendMessage, true);
        sendStatusMessageSPIFromSlave(wakeUpPin, mS_sendMessage);

        lcdMaster.clear();
        lcdMaster.setCursor(0, 0);

        if (radioIsWorking)
        {
          showMessage(lcdMaster, lcdMasterParams, 39);    //"Step 19         ","Radio com    ok ");
          digitalWrite(radioActiveLightPin, HIGH);
        }
        else
        {
          showMessage(lcdMaster, lcdMasterParams, 40);    //"Step 19         ","Radio com   NOK ");
          digitalWrite(radioActiveLightPin, LOW);
        }
        returnMessage = messageUnderstood;
        break;
      case vehicleCommandList:
        currentOS = activeOperationalState;
        for (int i = 0; i < mR_receiveMessage.totalContentItems; ++i)
        {
          inputOS = activeOperationalState;
          aux.topic = (messageCommands)mR_receiveMessage.content[i * 3];
          aux.parameter1 = mR_receiveMessage.content[i * 3 + 1];
          aux.parameter2 = mR_receiveMessage.content[i * 3 + 2];
          stateMessagecombination (inputOS, aux.topic, aux.parameter1, aux.parameter2);
        }
        displayEquipmentOrders();
        // ack to gs
        returnMessage = messageUnderstood;
        break;
      case noMessageActive:
        returnMessage = messageUnderstood;
        break;
      case searchOriginReply:
        originFoundIndicator = true;
        returnMessage = messageUnderstood;
        break;
      case locationRequest:
        sendCurrentLocationToSerial();
        returnMessage = messageUnderstood;
        break;
      case radioStatusRequest:
      case locationReply:
      case messageStatusReply:
      case vehicleStateList:
      case searchOriginRequest:
        returnMessage = messageUnknown;
        break;
      case errorSerialCommunication:
        debugSerialPrint(185, 185, true); //M-Type ID undefined
        _SERIAL_PRINTLN(mR_receiveMessage.messageTypeId);
        returnMessage = messageUnknown;
        // nack to gs
        break;
      default:
        returnMessage = messageUnknown;
        break;
    }
    reDisplay = true;
  }
  else
  {
    _SERIAL_PRINTLN("CRC status error");
    lcdMaster.clear();
    lcdMaster.print("CRCError");
  }

  //sendMessageAckNackToSerial(returnMessage, messageTypeId);
  // return to current operational state for execution

  return messageTypeId;
}

void checkForMessage()
{
  //
  if (hasData)
  {
    decodeSerialMessage();
    hasData = false;
  }
}

void writeLogLineOnFile (char logLine[80])
{
  char timeString[10];
  int characterPosition = 0;

  DateTime now = defineNow.now();
  sprintf(timeString, "%02i:%02i:%02i ", now.hour(), now.minute(), now.second());

  while (logLine[characterPosition] != '\0')
  {
    characterPosition++;
  }
  while (characterPosition < 80)
  {
    logLine[characterPosition] = ' ';
    characterPosition++;
  }

  TCA9548A(miniProEEPROMChannel);
  setCommunicationLED(callToI2CPin, LEDOn, 5);
  setCommunicationLED(callToI2CPin, LEDOff, 0);

  // part 1

  Wire.beginTransmission (DASHBOARD_ADDRESS);
  Wire.write(informLoglinePart1);
  for (int i = 0; i < 10; i++)
  {
    Wire.write(timeString[i]);
  }
  for (int i = 0; i < 20; i++)
  {
    Wire.write(logLine[i]);
  }
  Wire.endTransmission();
  delay(30);

  // part 2

  Wire.beginTransmission (DASHBOARD_ADDRESS);
  Wire.write(informLoglinePart2);
  for (int i = 20; i < 50; i++)
  {
    Wire.write(logLine[i]);
  }
  Wire.endTransmission();
  delay(30);

  // part 3

  Wire.beginTransmission (DASHBOARD_ADDRESS);
  Wire.write(informLoglinePart3);
  for (int i = 50; i < 71; i++)
  {
    Wire.write(logLine[i]);
  }
  Wire.endTransmission();
}

int checkWheelPulsesCount()
{
  int aux;
  TCA9548A(miniProEEPROMChannel);
  setCommunicationLED(callToI2CPin, LEDOn, 5);
  setCommunicationLED(callToI2CPin, LEDOff, 0);
  setCommunicationLED(waitForAnswerPin, LEDOn, 5);
  Wire.beginTransmission(0x0C); // transmit to device
  Wire.write(requestWheelPulsesCount);
  Wire.endTransmission();    // stop transmitting
  Wire.requestFrom(0x0C, 2);
  byte msb = Wire.read();
  byte lsb = Wire.read();
  _SERIAL_PRINT("msb:");
  _SERIAL_PRINT(msb);
  _SERIAL_PRINT(", lsb:");
  _SERIAL_PRINTLN(lsb);
  aux = ((int)msb << 8) + (int)lsb;
  setCommunicationLED(waitForAnswerPin, LEDOff, 0);
  return aux;
}

void stepsStraight(int straightDistance)
{
  char logLineBuffer[80];
  char textBlock[41];
  int pulsesPassed;

  //debugSerialPrint(193, 193, true); //"I: stepsStraight");

  memset (logLineBuffer, 0, sizeof(logLineBuffer));
  EETexts.readBlock (190, 32, true, textBlock); //move %d
  sprintf(logLineBuffer, textBlock, straightDistance);
  writeLogLineOnFile(logLineBuffer);
  lcdMaster.clear();
  showMessagePart (lcdMaster, lcdMasterParams, logLineBuffer, 0, 0);

  long starttime, timepassed;
  long starttimeDebug;
  returntoStatus = activeOperationalState;

  distanceTraveled = 0;
  debugSerialPrint(195, 195, false);//"Distance to travel:\t");
  _SERIAL_PRINTLN(straightDistance);
  executionStatus = executingMove;
  setDirectionIndicatorLights();
  sendMotorOrdersStraight(drivingDirection, 0, maxRPM, straightDistance);
  _SERIAL_PRINTLN("Start listening to mainMotorsOn");
  checkWheelPulseTimeOut = true;
  askForPulsesPassed = false;
  bool horizontalLine = true;
  starttimeDebug = millis();
  while (mainMotorsOn && ((millis() - starttimeDebug) < 10000))
  {
    checkSlaveData();
    if (askForPulsesPassed)
    {
      _SERIAL_PRINT("Pulsecount requested: ");
      pulsesPassed = checkWheelPulsesCount();
      _SERIAL_PRINTLN(pulsesPassed);
      askForPulsesPassed = false;
      horizontalLine = !horizontalLine;
      displayWheelPulses (lcdSlave, pulsesPassed, straightDistance, horizontalLine);
    }
  }
  horizontalLine = !horizontalLine;
  displayWheelPulses (lcdSlave, pulsesPassed, straightDistance, horizontalLine);

  checkWheelPulseTimeOut = false;
  askForPulsesPassed = false;
  pulsesPassed = checkWheelPulsesCount();
  //sendMotorOrdersStop();
  distanceTraveled = (float)pulsesPassed * distancePerPulse;
  executionStatus = executingStop;
  setDirectionIndicatorLights();
  displayWheelPulses (lcdSlave, pulsesPassed, straightDistance, horizontalLine);

  // debugSerialPrint(194, 194, true); //"O: stepsStraight");

}

float normalisedAngle(float rawHeading)
{
  float aux = rawHeading;
  if (rawHeading < 0.0)
  {
    aux = rawHeading + 360.0;
  }
  else if (rawHeading > 360.0)
  {
    aux = rawHeading - 360.0;
  }
  return aux;
}

void leftTurn(int turnAngle, int straightDistance)
{
  long startTurnTime;
  float steeringAngle;
  int steeringStepTime = 50;
  int maxTurnTime = 25000;
  int pulsesPassed = 0;
  char textBlock[33];
  float countAngles;
  float diffAngle;
  float previousHeading;
  float initialHeading;
  int restDistance = 0;
  float correctedTurnAngle;

  initialHeading = calculatedHeading;
  previousHeading = calculatedHeading;
  EETexts.readBlock (197, 32, true, textBlock); // 4, turn left
  writeLogLineOnFile(textBlock);
  debugSerialPrint(198, 198, false); //current heading
  _SERIAL_PRINT(currentHeading);
  _SERIAL_PRINT(", previousHeading = ");
  _SERIAL_PRINT(previousHeading);
  _SERIAL_PRINT(", turnAngle = ");
  _SERIAL_PRINT(turnAngle);

  countAngles = 0.0;
  (drivingDirection == drivingForwards) ? targetHeading = normalisedAngle(calculatedHeading - turnAngle) : targetHeading = normalisedAngle(calculatedHeading + turnAngle);
  debugSerialPrint(199, 199, false);//"\t targetHeading = ");
  _SERIAL_PRINTLN(targetHeading);

  executingLeftTurn = true;
  dashboardInfo = 2;
  sendIntToSlave(dashboardInfo, slaveNodeAddress);
  executionStatus = executingLeft;
  setDirectionIndicatorLights();
  startTurnTime = millis();
  checkWheelPulseTimeOut = true;
  askForPulsesPassed = false;
  showMessage(lcdMaster, lcdMasterParams, 198);
  displayTurnObjective(lcdMaster, currentHeading, targetHeading);
  steeringAngle = maxWheelAmplitude;
  bool horizontalLine = true;
  (drivingDirection == drivingForwards) ? steeringMotorGoLeft((int)steeringAngle) : steeringMotorGoRight((int)steeringAngle);

  //activateTurtle(9);  // rotate turtle 11 to the left (=9)
  sendMotorOrdersLeftTurn(drivingDirection, 0, maxRPM);
  correctedTurnAngle = abs(targetHeading - getHeading());
  _SERIAL_PRINT("correctedTurnAngle = ");
  _SERIAL_PRINTLN(correctedTurnAngle);

  do
  {
    _SERIAL_PRINT("(abs(countAngles) = ");
    _SERIAL_PRINTLN(abs(countAngles));
    currentHeading = getHeading();
    diffAngle = abs(currentHeading - previousHeading);
    if (diffAngle > 90)
    {
      diffAngle = 360 - diffAngle;
    }
    countAngles = countAngles + diffAngle;
    displayTurnProgress(lcdSlave, currentHeading, previousHeading, diffAngle, countAngles, correctedTurnAngle, horizontalLine);
    horizontalLine = !horizontalLine;
    previousHeading = currentHeading;

    if (angleCloseEnough(currentHeading, targetHeading))
    {
      countAngles = correctedTurnAngle + 1.0; // force stop turning
      _SERIAL_PRINTLN("Close-enough: Force stop turning");
    }

    if (millis() > (startTurnTime + maxTurnTime))
    {
      countAngles = turnAngle + 1.0;
    }
    else
    {
      if (abs(countAngles) > (int)correctedTurnAngle)
      {
        _SERIAL_PRINT("target and current heading: ");
        _SERIAL_PRINT(targetHeading);
        _SERIAL_PRINT(", ");
        _SERIAL_PRINT(currentHeading);

        if (!angleCloseEnough(targetHeading, currentHeading))
        {
          _SERIAL_PRINT(" => too far apart, angles reset to ");
          countAngles = min(abs(currentHeading - initialHeading), abs(abs(currentHeading - initialHeading) - 360));
          _SERIAL_PRINTLN(countAngles);
        }
        else
        {
          _SERIAL_PRINTLN(" => target reached ");
        }
      }
    }
    displayMotorData();
  }
  while (abs(countAngles) < (int)correctedTurnAngle);
  calculatedHeading = calculatedHeading - turnAngle;
  if (calculatedHeading < 0)
  {
    calculatedHeading = calculatedHeading + 360;
  }
  steeringMotorGoStraight(0);

  //activateTurtle(11);  // stop the turtle 11 (=0)
  pulsesPassed = checkWheelPulsesCount();
  restDistance = ((float)straightDistance - (float)pulsesPassed * distancePerPulse);
  if (restDistance > 0)
  {
    stepsStraight(restDistance);
  }
  //sendMotorOrdersStop();
  executingLeftTurn = false;
  executionStatus = executingStop;
  setDirectionIndicatorLights();
}

bool guidedDriveAdjustToTheRight()
{

  byte sensorState;
  float steeringAngle;
  bool continueMoving;
  bool seesLeftLine = false;
  bool seesRightLine = false;
  bool arrived;

  executingRightTurn = true;
  dashboardInfo = 32;
  sendIntToSlave(dashboardInfo, slaveNodeAddress);
  executionStatus = executingRight;
  setDirectionIndicatorLights();
  sendMotorOrdersRightTurn(drivingDirection, 0, maxRPM);
  showMessage(lcdMaster, lcdMasterParams, 198);
  steeringAngle = maxWheelAmplitude;
  (drivingDirection == drivingForwards) ? steeringMotorGoRight ((int)steeringAngle) : steeringMotorGoLeft ((int)steeringAngle);
  arrived = false;
  while (continueMoving)
  {
    (drivingDirection == drivingForwards) ? seesLeftLine = !digitalRead(leftFollowLineSensorPin) : seesLeftLine = !digitalRead(rightFollowLineSensorPin);
    continueMoving = seesLeftLine;
    if (continueMoving)
    {
      (drivingDirection == drivingForwards) ? seesRightLine = !digitalRead(rightFollowLineSensorPin) : seesRightLine = !digitalRead(leftFollowLineSensorPin);
      if (seesRightLine)
      {
        continueMoving = false;// now both sensors see a black line ... so stop
        arrived = true;
      }
    }
  }

  return arrived;
}
bool guidedDriveAdjustToTheLeft()
{
  byte sensorState;
  float steeringAngle;
  bool continueMoving;
  bool seesLeftLine = false;
  bool seesRightLine = false;
  bool arrived;

  executingRightTurn = true;
  dashboardInfo = 32;
  sendIntToSlave(dashboardInfo, slaveNodeAddress);
  executionStatus = executingRight;
  setDirectionIndicatorLights();
  sendMotorOrdersLeftTurn(drivingDirection, 0, maxRPM);
  showMessage(lcdMaster, lcdMasterParams, 198);
  steeringAngle = maxWheelAmplitude;
  (drivingDirection == drivingForwards) ? steeringMotorGoLeft ((int)steeringAngle) : steeringMotorGoRight ((int)steeringAngle);
  continueMoving = true;
  arrived = false;
  while (continueMoving)
  {
    (drivingDirection == drivingForwards) ? seesRightLine = !digitalRead(rightFollowLineSensorPin) : seesRightLine = !digitalRead(leftFollowLineSensorPin);
    continueMoving = seesRightLine;
    if (continueMoving)
    {
      (drivingDirection == drivingForwards) ? seesLeftLine = !digitalRead(leftFollowLineSensorPin) : seesLeftLine = !digitalRead(rightFollowLineSensorPin);
      if (seesLeftLine)
      {
        continueMoving = false;// now both sensors see a black line ... so stop
        arrived = true;;
      }
    }
  }

  return arrived;
}

void rightTurn(int turnAngle, int straightDistance)
{
  long  startTurnTime;
  float steeringAngle;
  int   steeringStepTime = 125;
  int   maxTurnTime = 25000;
  char  textBlock[33];
  float countAngles;
  float diffAngle;
  float previousHeading;
  float initialHeading;
  int restDistance = 0;
  int pulsesPassed;
  float correctedTurnAngle;

  initialHeading = calculatedHeading;
  previousHeading = calculatedHeading;
  EETexts.readBlock (200, 32, true, textBlock); // 4, turn right
  writeLogLineOnFile(textBlock);
  debugSerialPrint(198, 198, false); //current heading
  _SERIAL_PRINT(currentHeading);
  _SERIAL_PRINT(", previousHeading = ");
  _SERIAL_PRINT(previousHeading);
  _SERIAL_PRINT(", turnAngle = ");
  _SERIAL_PRINT(turnAngle);

  countAngles = 0.0;
  (drivingDirection == drivingForwards) ? targetHeading = normalisedAngle(calculatedHeading + turnAngle) : targetHeading = normalisedAngle(calculatedHeading - turnAngle);
  debugSerialPrint(199, 199, false);//"\t targetHeading = ");
  _SERIAL_PRINTLN(targetHeading);

  executingRightTurn = true;
  dashboardInfo = 32;
  sendIntToSlave(dashboardInfo, slaveNodeAddress);
  executionStatus = executingRight;
  setDirectionIndicatorLights();
  startTurnTime = millis();
  checkWheelPulseTimeOut = true;
  askForPulsesPassed = false;
  showMessage(lcdMaster, lcdMasterParams, 198);
  displayTurnObjective(lcdMaster, currentHeading, targetHeading);
  steeringAngle = maxWheelAmplitude;
  bool horizontalLine = true;
  (drivingDirection == drivingForwards) ? steeringMotorGoRight ((int)steeringAngle) : steeringMotorGoLeft ((int)steeringAngle);

  //activateTurtle(10); // start turtle move to the right (=10);
  sendMotorOrdersRightTurn(drivingDirection, 0, maxRPM);
  correctedTurnAngle = abs(targetHeading - getHeading());
  _SERIAL_PRINT("correctedTurnAngle = ");
  _SERIAL_PRINTLN(correctedTurnAngle);

  do
  {
    _SERIAL_PRINT("(abs(countAngles) = ");
    _SERIAL_PRINTLN(abs(countAngles));
    currentHeading = getHeading();
    diffAngle = abs(previousHeading -  currentHeading);
    if (diffAngle > 90)
    {
      diffAngle = 360 - diffAngle;
    }
    countAngles = countAngles + diffAngle;
    displayTurnProgress(lcdSlave, currentHeading, previousHeading, diffAngle, countAngles, correctedTurnAngle, horizontalLine);
    horizontalLine = !horizontalLine;
    previousHeading = currentHeading;

    if (angleCloseEnough(currentHeading, targetHeading))
    {
      countAngles = turnAngle + 1.0; // force stop turning
      _SERIAL_PRINTLN("Close-enough: Force stop turning");
    }

    if (millis() > (startTurnTime + maxTurnTime))
    {
      countAngles = turnAngle + 1.0;
    }
    else
    {
      if (abs(countAngles) > (int)correctedTurnAngle)
      {
        _SERIAL_PRINT("target and current heading: ");
        _SERIAL_PRINT(targetHeading);
        _SERIAL_PRINT(", ");
        _SERIAL_PRINT(currentHeading);

        if (!angleCloseEnough(targetHeading, currentHeading))
        {
          _SERIAL_PRINT(" => too far apart, angles reset to ");
          countAngles = min(abs(currentHeading - initialHeading), abs(abs(currentHeading - initialHeading) - 360));
          _SERIAL_PRINTLN(countAngles);
        }
        else
        {
          _SERIAL_PRINTLN(" => target reached ");
        }
      }
    }
    displayMotorData();
  }
  while (abs(countAngles) < (int)correctedTurnAngle);
  calculatedHeading = calculatedHeading + turnAngle;
  if (calculatedHeading > 360)
  {
    calculatedHeading = calculatedHeading - 360;
  }

  steeringMotorGoStraight(0);
  //activateTurtle(11); // stop turtle (=10);
  pulsesPassed = checkWheelPulsesCount();
  restDistance = ((float)straightDistance - (float)pulsesPassed * distancePerPulse);
  if (restDistance > 0)
  {
    stepsStraight(restDistance);
  }
  //sendMotorOrdersStop();
  executingRightTurn = false;
  executionStatus = executingStop;
  setDirectionIndicatorLights();

}


void stateMessagecombination ( operationPossibilities state, messageCommands messageCode, int messageParameter1, int messageParameter2)
{
  debugSerialPrint(201, 202, false);//"I: stateMessagecombination; Status: ");
  _SERIAL_PRINT(state);
  debugSerialPrint(203, 203, false);//"\t message: ");
  _SERIAL_PRINTLN(messageCode);

  operationPossibilities operationalStateToOrderList;
  activeOperationalState = abortedRunMode;
  // no state change possible in the following states : stopped, start driving, turn left, turn right, accelerate, decelerate, and locate vehicle
  switch (state)
  {
    case switchedOffMode:
      activeOperationalState = getNextActiveOperationalState();
      break;
    case standByMode:
      switch (messageCode) {
        case switchToStandByMode:
          operationalStateToOrderList = standByMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case executeCommands:
          activeOperationalState = executeCommandsMode;
          break;
        case switchToForwardDriving:
          operationalStateToOrderList = forwardDrivingMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case switchToBackwardDriving:
          operationalStateToOrderList = backwardDrivingMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case switchOff:
          operationalStateToOrderList = switchedOffMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case goStraight:
          operationalStateToOrderList = straightDrivingMode;
          addEquipmentOrderSegment (operationalStateToOrderList, messageParameter1, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case switchToManualMode:
          operationalStateToOrderList = startDrivingMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0 );
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case requestPosition:
          operationalStateToOrderList = checkPositionMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case switchToTestMode:
          operationalStateToOrderList = testMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case rotateRadarPole:
          operationalStateToOrderList = rotatePoleMode;
          addEquipmentOrderSegment (operationalStateToOrderList, messageParameter1, messageParameter2, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case moveSpreaderToTop:
          operationalStateToOrderList = liftSpreaderToTopMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case moveSpreaderToBottom:
          operationalStateToOrderList = dropSpreaderToBottomMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case lockSpreader:
          operationalStateToOrderList = lockSpreaderMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case unlockSpreader:
          operationalStateToOrderList = unlockSpreaderMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case goLeft:
          operationalStateToOrderList = turnLeftMode;
          addEquipmentOrderSegment (operationalStateToOrderList, messageParameter1, messageParameter2, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case goRight:
          operationalStateToOrderList = turnRightMode;
          addEquipmentOrderSegment (operationalStateToOrderList, messageParameter1, messageParameter2, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case switchToLocalGuiding:
          operationalStateToOrderList = guidedDriveMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
        case stopImmediately:
          removeAllEquipmentOrders();
          //addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          //activeOperationalState = getNextActiveOperationalState();
          break;
      }
      if (activeOperationalState != executeCommandsMode)
      {
        activeOperationalState = standByMode;
      }
      break;
    case continueDrivingMode:
      switch (messageCode) {
        case stopImmediately:
          removeAllEquipmentOrders();
          activeOperationalState = standByMode;
          break;
      }
      break;
    case switchToManualMode:
      switch (messageCode) {
        case goStraight:
          operationalStateToOrderList = straightDrivingMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 10, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case moveSpreaderToTop:
          operationalStateToOrderList = liftSpreaderToTopMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case moveSpreaderToBottom:
          operationalStateToOrderList = dropSpreaderToBottomMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case lockSpreader:
          operationalStateToOrderList = lockSpreaderMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case unlockSpreader:
          operationalStateToOrderList = unlockSpreaderMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case stopImmediately:
          operationalStateToOrderList = abortedRunMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
      }
      break;
    case testMode:
      switch (messageCode) {
        case goStraight:
          operationalStateToOrderList = straightDrivingMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 10, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case switchToStandByMode:
          operationalStateToOrderList = standByMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case moveSpreaderToTop:
          operationalStateToOrderList = liftSpreaderToTopMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case moveSpreaderToBottom:
          operationalStateToOrderList = dropSpreaderToBottomMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case lockSpreader:
          operationalStateToOrderList = lockSpreaderMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case unlockSpreader:
          operationalStateToOrderList = unlockSpreaderMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
        case stopImmediately:
          operationalStateToOrderList = abortedRunMode;
          addEquipmentOrderSegment (operationalStateToOrderList, 0, 0, 0);
          activeOperationalState = getNextActiveOperationalState();
          break;
      }
  }
  debugSerialPrint(204, 205, false);//"O: stateMessageCombination with Status: ");
  _SERIAL_PRINTLN(activeOperationalState);
}

void sendIntToSlave(int number, int slaveId)
{
  _SERIAL_PRINT("SendIntToSlave met dashboardInfo: ");
  _SERIAL_PRINTLN(dashboardInfo);
  if (checkI2Cdevice(slaveId, mainI2CChannel) == 0)
  {

    setCommunicationLED(callToI2CPin, LEDOn, 5);
    TCA9548A(mainI2CChannel);

    Wire.beginTransmission(slaveId);
    Wire.write(informToReceiveDashboardInfo);
    Wire.endTransmission();
    setCommunicationLED(callToI2CPin, LEDOff, 0);

    setCommunicationLED(callToI2CPin, LEDOn, 5);
    delay(40);
    Wire.beginTransmission(slaveId);
    Wire.write((byte)number);
    Wire.endTransmission();
    setCommunicationLED(callToI2CPin, LEDOff, 0);
  }
}

void triggerFromSpreader()
{
  spreaderMotorOn = false;
  _SERIAL_PRINTLN("Trigger spreader received");
}
void obstacleDetected()
{
  activeOperationalState = obstacleAvoidanceMode;
  soundSiren(3000);
}

void locateVehicle()
{

  //
  // rotate pole engine 360 degrees and read ToF sensor each smallest step
  //
  // step 1: determine arc to origin and distance from origin from the perspective of the vehicle driving direction (ABC) plus distance
  // name ABC = originAngleRelativeToVehicle and correct it for the misalignment and wide field of vision of the laser
  // name l = length = distanceInMM
  if (delayFactor == 1)
  {
    currentPosition.orientationRelativeToGrid = 0;
    currentPosition.x = 100;
    currentPosition.y = 100;
  }
  else
  {
    _SERIAL_PRINTLN("In locateVehicle");
    digitalWrite(laserPointerPin, HIGH);
    alignPoleMotorClockWise();
    poleMotorFindOrigin(0);
    debugSerialPrint(207, 207, true);//"Locate Step 1: ");
    debugSerialPrint(208, 208, false);//"(ABC)");
    _SERIAL_PRINT(originAngleRelativeToVehicle);
    originAngleRelativeToVehicle = originAngleRelativeToVehicle - correctionOfLaserposition;
    debugSerialPrint(209, 209, false);//"\t after correction");
    _SERIAL_PRINT(originAngleRelativeToVehicle);
    debugSerialPrint(210, 210, false);//"\t mm: ");
    _SERIAL_PRINTLN(distanceInMM);
    // step 2: get compassreading (CBE)
    // name CBE = orientationVehicleRelativeToNorth
    orientationVehicleRelativeToNorth = getHeading();
    debugSerialPrint(211, 211, true);//"Locate Step 2: ");
    debugSerialPrint(212, 212, false);//"(CBE)");
    _SERIAL_PRINTLN(orientationVehicleRelativeToNorth);
    //
    // step 3: DBE = orientationOfGridRelativeToNorth already determined
    debugSerialPrint(213, 213, true);//"Locate Step 3: ");
    debugSerialPrint(214, 214, false);//"(DBE)");
    _SERIAL_PRINTLN(orientationOfGridRelativeToNorth);
    // step 4: Determine arc between vehicle and origine on grid orientation (ABD = ABC - DBE + CBE)
    double originAngleRelativeToGrid = originAngleRelativeToVehicle - (orientationVehicleRelativeToNorth - orientationOfGridRelativeToNorth);
    debugSerialPrint(215, 215, true); //"Locate Step 4:
    debugSerialPrint(216, 216, false);//"(ABD)");
    _SERIAL_PRINTLN(originAngleRelativeToGrid);
    //step 5: Determine (x,y) of origin from the perspective of the vehicle
    double originXfromVehicle = cos((originAngleRelativeToGrid) * PI / 180.0) * distanceInMM;
    double originYfromVehicle = sin((originAngleRelativeToGrid) * PI / 180.0) * distanceInMM;
    debugSerialPrint(217, 217, true); //"Locate Step 5:");
    debugSerialPrint(218, 218, false);//" V(x,y)(");
    _SERIAL_PRINT(originXfromVehicle);
    _SERIAL_PRINT (",");
    _SERIAL_PRINT(originYfromVehicle);
    _SERIAL_PRINT  (")");
    // step 6: determine position of Vehicle from the perspective of the grid (with the grid x-axis in the driving direction of the vehicle)
    double vehicleXfromOrigin = ZeroRadarX - originXfromVehicle;
    double vehicleYfromOrigin = ZeroRadarY - originYfromVehicle;
    debugSerialPrint(219, 219, true); //"Locate Step 6:");
    debugSerialPrint(220, 220, false);//" O(x,y)(");
    _SERIAL_PRINT(vehicleXfromOrigin);
    _SERIAL_PRINT (",");
    _SERIAL_PRINT(vehicleYfromOrigin);
    _SERIAL_PRINTLN (")");

    // step 7: determine current position and orientation of the vehicle relative to the grid
    currentPosition.orientationRelativeToGrid = (360 + ((int)originAngleRelativeToGrid)) % 360;
    currentPosition.x = vehicleXfromOrigin;
    currentPosition.y = vehicleYfromOrigin;

    debugSerialPrint(221, 222, false); //"Locate Step 7: position and Orientation of vehicle on grid: ");
    _SERIAL_PRINTLN(currentPosition.orientationRelativeToGrid);
    _SERIAL_PRINT("(");
    _SERIAL_PRINT(currentPosition.x);
    _SERIAL_PRINT(", ");
    _SERIAL_PRINT(currentPosition.y);
    _SERIAL_PRINTLN(")");

    //
    // return laser to look straight ahead for next positioning request
    //
    alignPoleMotorClockWise();
    digitalWrite(laserPointerPin, LOW);
  }
}

void poleMotorAntiClockWise(int rotateDegrees)
{
  _SERIAL_PRINTLN("in poleMotorAntiClockWise");
  int aux = (float)rotateDegrees * uniPolarstepperMotoronedegree;

  for (int j = 0; j < aux; ++j)
  {
    for (int i = 7; i >= 0; i--)
    {
      setOutput(i);
      delayMicroseconds(poleMotorSpeed);
    }
  }
  digitalWrite(poleMotorPin1, LOW);
  digitalWrite(poleMotorPin2, LOW);
  digitalWrite(poleMotorPin3, LOW);
  digitalWrite(poleMotorPin4, LOW);
  poleMotorOrientationRelativeToVehicle = poleMotorOrientationRelativeToVehicle - rotateDegrees;
}

void alignPoleMotorClockWise()
{
  bool aligned = false;
  int oldphotoSensorValue = -1;
  int photoSensorValue = 0;
  bool lightFound = false;
  int lightFoundCount = 0;
  // debugSerialPrint(, false);//"I: alignPoleMotorClockWise ");
  int minimumDistance;
  int margin;
  int distanceMeasured;

  digitalWrite(lightSourcePin, HIGH);
  while (!aligned)
  {
    for (int i = 0; i < 8; ++i)
    {
      setOutput(i);
      delayMicroseconds(1800);
    }
    photoSensorValue = analogRead(lightSensorPin);
    if (lightFoundCount == 0)
    {
      if (!lightFound)
      {
        lightFound = (photoSensorValue > 30);
      }
      else
      {
        if (photoSensorValue < 15)
        {
          lightFoundCount = 1;
          lightFound = false;
        }
      }
    }
    else
    {
      if (photoSensorValue > 30)
      {
        aligned = true;
      }
    }

    oldphotoSensorValue = photoSensorValue;
  }
  digitalWrite(poleMotorPin1, LOW);
  digitalWrite(poleMotorPin2, LOW);
  digitalWrite(poleMotorPin3, LOW);
  digitalWrite(poleMotorPin4, LOW);
  digitalWrite(lightSourcePin, LOW);

  poleMotorOrientationRelativeToVehicle = 0;
}

void poleMotorClockWise(int rotateDegrees)
{
  int aux = (float)rotateDegrees * uniPolarstepperMotoronedegree;

  for (int j = 0; j < aux; ++j)
  {
    for (int i = 0; i < 8; ++i)
    {
      setOutput(i);
      delayMicroseconds(poleMotorSpeed);
    }
  }
  digitalWrite(poleMotorPin1, LOW);
  digitalWrite(poleMotorPin2, LOW);
  digitalWrite(poleMotorPin3, LOW);
  digitalWrite(poleMotorPin4, LOW);
  poleMotorOrientationRelativeToVehicle = poleMotorOrientationRelativeToVehicle + rotateDegrees;

}

bool reliableDistance()
{
  bool aux;

  distanceInMM = toFSensor.readSingle();

  if (toFSensor.timeoutOccurred())
  {
    debugSerialPrint(223, 223, true); //" toFSensor TIMEOUT");
    aux = false;
  }
  else
  {
    debugSerialPrint(224, 224, false);//"range: ");
    _SERIAL_PRINT(toFSensor.ranging_data.range_mm);
    debugSerialPrint(226, 226, false);//"\tpeak signal: ");
    _SERIAL_PRINT(toFSensor.ranging_data.peak_signal_count_rate_MCPS);
    debugSerialPrint(227, 227, false);//"\tambient: ");
    _SERIAL_PRINT(toFSensor.ranging_data.ambient_count_rate_MCPS);
    debugSerialPrint(225, 225, false);//"\t, status: ");
    int measurementStatus = toFSensor.ranging_data.range_status;
    _SERIAL_PRINTLN(VL53L1X::rangeStatusToString(toFSensor.ranging_data.range_status));
    aux = measurementStatus |= 4;// see documentation ST (VL53L1X range status)
  }
  //
  //
  return aux;
}

void activateRadar()
{
  byte rawContent[maxMessageParameters];

  _SERIAL_PRINTLN("in activateRadar");
  memset (rawContent, 0, sizeof(rawContent));
  rawContent[0] = (byte)searchForOrigin;
  rawContent[1] = 0;
  rawContent[2] = 0;

  if (radioIsWorking)
  {
    messageNumber++;
    mS_sendMessage = prepareSerialMessage (straddleName + "M", straddleName + "R", pointZeroName + "R", searchOriginRequest, 1, &rawContent[0]);
    mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
  }
}

void activateTurtle(byte turtleAction)
{
  byte rawContent[maxMessageParameters];

  _SERIAL_PRINTLN("in activateTurtle");
  memset (rawContent, 0, sizeof(rawContent));
  rawContent[0] = (byte)mecanumDrive;
  rawContent[1] = (byte)turtleAction;
  rawContent[2] = 0;
  rawContent[3] = (byte)11;
  rawContent[4] = 0;
  rawContent[5] = 0;

  if (radioIsWorking)
  {
    messageNumber++;
    mS_sendMessage = prepareSerialMessage (straddleName + "M", straddleName + "R",  equipmentNameList[5].substring(0, 4) + "M", vehicleCommandList, 2, &rawContent[0]);
    mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
    displayMessageLCD(mS_sendMessage, sent, 0);
  }
}

float findOriginPole(int motorSpeed)
{
  float aux;
  originFoundIndicator = false;
  bool measurementReliable;
  int j = 0;
  int i = 0;
  String indicator;
  _SERIAL_PRINTLN("in findOriginPole");
  activateRadar();
  if (tofSensorStatus == operational)
  {
    while (!originFoundIndicator)
    {
      i = 0;
      while ((i < 8) && (!originFoundIndicator))
      {
        checkForMessage();
        i++;
        if (!originFoundIndicator)
        {
          setOutput(i);
          delayMicroseconds(motorSpeed);
        }
      }
      // orientation is 1 step positive so 1/(512) th of 360 degrees
      ++j;
      // if searchlight went round one whole time, resend the activate radar request and try again
      if (j == 512)
      {
        activateRadar();
        j = 0;
      }
    }
    aux = float(j - 1) * uniPolarstepperMotorDegreesPerStep + float(i) * (360.0 / 4192.0);
  }
  else
  {
    aux = 0;
  }
  TCA9548A(miniProEEPROMChannel);
  measurementReliable = reliableDistance();
  _SERIAL_PRINTLN("after distance measurement");
  lcdSlave.clear();
  lcdSlave.setCursor(0, 0);
  lcdSlave.print("Distance:");
  lcdSlave.setCursor(9, 0);
  lcdSlave.print(distanceInMM);
  lcdSlave.setCursor(0, 1);
  lcdSlave.print("Angle:");
  lcdSlave.setCursor(9, 1);
  lcdSlave.print(aux);
  return aux;
}

void poleMotorFindOrigin(int startAngleRelativeToVehicle)
{
  bool measurementReliable;
  float roughMeasurement;
  _SERIAL_PRINTLN("In poleMotorFindOrigin");

  originAngleRelativeToVehicle = 0.0;
  distanceInMM = 0;

  poleMotorAntiClockWise(startAngleRelativeToVehicle);
  digitalWrite(laserPointerPin, HIGH);
  roughMeasurement = findOriginPole(9000);
  originAngleRelativeToVehicle = roughMeasurement - startAngleRelativeToVehicle;
  debugSerialPrint(234, 234, false);//"leading edge at: ");
  _SERIAL_PRINT(originAngleRelativeToVehicle);
  debugSerialPrint(235, 235, true); //" relative to vehicle ");
  digitalWrite(poleMotorPin1, LOW);
  digitalWrite(poleMotorPin2, LOW);
  digitalWrite(poleMotorPin3, LOW);
  digitalWrite(poleMotorPin4, LOW);
  waitFor(100);
  digitalWrite(laserPointerPin, LOW);
}

void setOutput(int out)
{
  (bitRead(lookup[out], 0) == 1) ? digitalWrite(poleMotorPin1, HIGH) : digitalWrite(poleMotorPin1, LOW);
  (bitRead(lookup[out], 1) == 1) ? digitalWrite(poleMotorPin2, HIGH) : digitalWrite(poleMotorPin2, LOW);
  (bitRead(lookup[out], 2) == 1) ? digitalWrite(poleMotorPin3, HIGH) : digitalWrite(poleMotorPin3, LOW);
  (bitRead(lookup[out], 3) == 1) ? digitalWrite(poleMotorPin4, HIGH) : digitalWrite(poleMotorPin4, LOW);
}

void steeringMotorGoStraight(int crabAngle)
{
  TCA9548A(miniProEEPROMChannel);
  setServoToAngle(leftFront, 90 + crabAngle);
  setServoToAngle(leftRear, 90 + crabAngle);
  setServoToAngle(rightFront, 90 + crabAngle);
  setServoToAngle(rightRear, 90 + crabAngle);
}

void steeringMotorGoRight(int rotateDegrees)
{
  TCA9548A(miniProEEPROMChannel);
  setServoToAngle(leftFront, 90 + (int)(differentialTurnFraction * float(rotateDegrees))); // frontwheels pointing to the right
  setServoToAngle(rightFront, 90 + rotateDegrees);
  setServoToAngle(leftRear, 90 - (int)(differentialTurnFraction * float(rotateDegrees))); // rearwheels pointing to the left
  setServoToAngle(rightRear, 90 - rotateDegrees);
}

void steeringMotorGoLeft(int rotateDegrees)
{
  TCA9548A(miniProEEPROMChannel);
  setServoToAngle(leftFront, 90 - rotateDegrees);
  setServoToAngle(rightFront, 90 - (int)(differentialTurnFraction * float(rotateDegrees)));  // frontwheels pointing to the left
  setServoToAngle(leftRear, 90 + rotateDegrees);
  setServoToAngle(rightRear, 90 + (int)(differentialTurnFraction * float(rotateDegrees)));  // rearwheels pointing to the right
}

void liftSpreader()
{
  // used for lifting the spreader on to the top position
  // trigger monitoring the distance of the spreader from the platform
  int spreaderHeight;
  bool line = false;

  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(mainI2CChannel);
  Wire.beginTransmission(slaveNodeAddress);
  Wire.write(requestToLiftSpreader);
  Wire.endTransmission();
  setCommunicationLED(callToI2CPin, LEDOff, 0);

  analogWrite(motorSpreaderEnablePin, 0);
  digitalWrite(motorSpreaderPin1, LOW);
  digitalWrite(motorSpreaderPin2, HIGH);
  spreaderMotorOn = true;
  checkSpreaderHeight = true;

  analogWrite(motorSpreaderEnablePin, 255);
  while (spreaderMotorOn)
  {
    if (askForSpreaderHeight)
    {
      spreaderHeight = fetchSpreaderHeight();
      displayHoistProgress (lcdMotors, spreaderHeight, line);
      line != line;
      askForSpreaderHeight = false;
    }
    delay(10);
  }
  checkSpreaderHeight = false;

  analogWrite(motorSpreaderEnablePin, 0);
  digitalWrite(motorSpreaderPin1, LOW);
  digitalWrite(motorSpreaderPin2, LOW);

}

void dropSpreader()
{
  // used for dropping the spreader on to the pick-up or set down position
  int spreaderHeight;
  bool line = false;
  double aux;
  const double mmPerRotation = 65.97;

  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(mainI2CChannel);
  Wire.beginTransmission(slaveNodeAddress);
  Wire.write(requestToDropSpreader);
  Wire.endTransmission();
  setCommunicationLED(callToI2CPin, LEDOff, 0);

  analogWrite(motorSpreaderEnablePin, 0);
  digitalWrite(motorSpreaderPin1, HIGH);
  digitalWrite(motorSpreaderPin2, LOW);
  spreaderMotorOn = true;
  checkSpreaderHeight = true;
  analogWrite(motorSpreaderEnablePin, 255);
  while (spreaderMotorOn)
  {
    if (askForSpreaderHeight)
    {
      spreaderHeight = fetchSpreaderHeight();
      displayHoistProgress (lcdMotors, spreaderHeight, line);
      line != line;
      askForSpreaderHeight = false;
    }
    delay(10);
  }
  checkSpreaderHeight = false;
  analogWrite(motorSpreaderEnablePin, 0);
  digitalWrite(motorSpreaderPin1, LOW);
  digitalWrite(motorSpreaderPin2, LOW);
}

byte checkI2Cdevice(int addressToCheck, uint8_t I2CChannel)
{

  byte error;

  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(I2CChannel);

  Wire.beginTransmission(addressToCheck);
  error = Wire.endTransmission();
  setCommunicationLED(callToI2CPin, LEDOff, 0);
  return error;

}

void setDrivingDirection (int movingDirection)
{
  if (drivingDirection != movingDirection)
  {
    drivingDirection = movingDirection;
    setDirectionIndicatorLights();
  }
}

int fetchSpreaderHeight()
{
  // function used to determine if spreader is at right hight to execute lift or drop order
  int aux = 0;
  if (checkI2Cdevice(slaveNodeAddress, mainI2CChannel) == 0)
  {
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    setCommunicationLED(waitForAnswerPin, LEDOn, 5);
    waitFor(40);
    TCA9548A(mainI2CChannel);
    Wire.beginTransmission(slaveNodeAddress);
    Wire.write(requestSpreaderHeight);
    Wire.endTransmission();
    Wire.requestFrom(slaveNodeAddress, 1);
    aux = Wire.read();
    setCommunicationLED(waitForAnswerPin, LEDOff, 0);
  }

  return aux;

}

void sendCurrentLocationToSerial()
{
  byte rawContent[14];

  if (radioIsWorking)
  {
    rawContent[0]  = (currentPosition.x & 0xFF00) >> 8;
    rawContent[1]  = (currentPosition.x & 0x00FF);
    rawContent[2]  = 0;

    rawContent[3]  = (currentPosition.y & 0xFF00) >> 8;
    rawContent[4]  = (currentPosition.y & 0x00FF);
    rawContent[5]  = 0;

    rawContent[6]  = (currentPosition.orientationRelativeToGrid & 0xFF00) >> 8;
    rawContent[7]  = (currentPosition.orientationRelativeToGrid & 0x00FF);
    rawContent[8]  = 0;

    rawContent[9]  = 0;
    rawContent[10] = 0;
    rawContent[11] = 0;

    messageNumber++;
    mS_sendMessage = prepareSerialMessage (straddleName + "M", straddleName + "R", groundStationName + "M", locationReply, 4, &rawContent[0]);
    mR_receiveMessage = sendMessageSPIFromSlave(wakeUpPin, mS_sendMessage);
  }
}

void displayMotorData()
{
  char lcdLine[21];

  if (((millis() - lastMotorDisplayTime) > displayMotorRefreshTime) && vehicleMoving)
  {
    lcdMotors.init();
    memset(lcdLine, 0, sizeof(lcdLine));
    TCA9548A(miniProEEPROMChannel);
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    setCommunicationLED(waitForAnswerPin, LEDOn, 5);

    Wire.beginTransmission(0x0B); // transmit to device
    Wire.write(requestDisplayMotorData);
    Wire.endTransmission();    // stop transmitting
    Wire.requestFrom(0x0B, 20);
    if (Wire.available() <= 20)
    {
      for ( int i = 0; i < 20; i++)
      {
        lcdLine[i] = Wire.read();
      }
    }
    setCommunicationLED(waitForAnswerPin, LEDOff, 0);
    showMessagePart (lcdMotors, lcdMotorsParams, lcdLine, 0, 0);

    memset(lcdLine, 0, sizeof(lcdLine));
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    setCommunicationLED(waitForAnswerPin, LEDOn, 5);
    TCA9548A(mainI2CChannel);

    Wire.beginTransmission(0x0C); // transmit to device
    Wire.write(requestDisplayMotorData);
    Wire.endTransmission();    // stop transmitting
    Wire.requestFrom(0x0C, 20);
    if (Wire.available() <= 20)
    {
      for ( int i = 0; i < 20; i++)
      {
        lcdLine[i] = Wire.read();
      }
    }
    setCommunicationLED(waitForAnswerPin, LEDOff, 0);

    showMessagePart (lcdMotors, lcdMotorsParams, lcdLine, 0, 1);

    memset(lcdLine, 0, sizeof(lcdLine));
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    setCommunicationLED(waitForAnswerPin, LEDOn, 5);
    TCA9548A(mainI2CChannel);

    Wire.beginTransmission(0x0D); // transmit to device
    Wire.write(requestDisplayMotorData);
    Wire.endTransmission();    // stop transmitting
    Wire.requestFrom(0x0D, 20);
    if (Wire.available() <= 20)
    {
      for ( int i = 0; i < 20; i++)
      {
        lcdLine[i] = Wire.read();
      }
      lcdLine[20] = 0;
    }
    setCommunicationLED(waitForAnswerPin, LEDOff, 0);
    showMessagePart (lcdMotors, lcdMotorsParams, lcdLine, 0, 2);

    lastMotorDisplayTime = millis();
  }
}

void resetSlave()
{
  debugSerialPrint(274, 274, true);//"ResetSlave sent");
  setCommunicationLED(callToI2CPin, LEDOn, 5);
  TCA9548A(mainI2CChannel);
  Wire.beginTransmission(slaveNodeAddress);
  Wire.write(requestResetSlave);
  Wire.endTransmission();
  setCommunicationLED(callToI2CPin, LEDOff, 0);
  waitFor(2000);
}

void displaySlaveMessage ()
{
  char line[21];
  for (int j = 0; j < 2; j++)
  {
    memset(line, 0, sizeof(line));
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    setCommunicationLED(waitForAnswerPin, LEDOn, 0);
    TCA9548A(mainI2CChannel);
    Wire.beginTransmission(slaveNodeAddress);
    Wire.write(requestDisplayLines);
    Wire.endTransmission();
    Wire.requestFrom(slaveNodeAddress, 16);
    int i = 0;
    while (Wire.available())
    {
      line[i] = Wire.read();
      i++;
    }
    setCommunicationLED(waitForAnswerPin, LEDOff, 0);
    line[16] = 0;
    lcdSlave.init();
    lcdSlave.setCursor(0, j);
    lcdSlave.print(line);
  }
}

void checkSlaveData()
{
  byte aux;

  if (triggerReceivedFromSlave)
  {
    setCommunicationLED(callFromInterruptPin, LEDOff, 0);
    setCommunicationLED(callToI2CPin, LEDOn, 5);
    setCommunicationLED(callToI2CPin, LEDOff, 0);
    setCommunicationLED(waitForAnswerPin, LEDOn, 5);
    TCA9548A(mainI2CChannel);
    Wire.beginTransmission(slaveNodeAddress);
    Wire.write(requestInterruptType);
    Wire.endTransmission();

    Wire.requestFrom(slaveNodeAddress, 1);
    aux = Wire.read();
    _SERIAL_PRINT("interrupt type: ");
    _SERIAL_PRINTLN(aux);
    switch (aux)
    {
      case slaveReady:
        slaveInitialising = false;
        break;
      case obstacleInterrupt: // 97 = obstacle detected
        obstacleDetected();
        mainMotorsOn = false;
        break;
      case displayLineInterrupt: // 96 = logline will follow
        displaySlaveMessage();
        break;
      case logLineInterrupt: // 96 = logline will follow
        //writeSlaveLogLine();
        break;
    }
    triggerReceivedFromSlave = false;
    setCommunicationLED(waitForAnswerPin, LEDOff, 0);
  }
}

void loop()
{
  char logLineBuffer[80];

  displayMotorData();
  checkSlaveData();

  switch (activeOperationalState)
  {
    case executeCommandsMode:
      _SERIAL_PRINTLN("in executeCommandsMode, next operation is: ");
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("execute command");
      calculatedHeading = getHeading();
      operationsActiveMode = true;
      previousActiveOperationalState = activeOperationalState;
      displayEquipmentOrders();
      activeOperationalState = getNextActiveOperationalState();
      displayEquipmentOrders();
      _SERIAL_PRINTLN(activeOperationalState);
      break;
    case abortedRunMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("aborted");
      sendMotorOrdersStop();
      digitalWrite(steeringMotorPowerSwitch, LOW);
      spreaderIsMoving = false;
      debugSerialPrint(242, 242, true); //"In abortedRun");
      spreaderLightsOff();
      dashboardInfo = 255;
      sendIntToSlave(dashboardInfo, slaveNodeAddress);
      logPosition();
      removeAllEquipmentOrders();
      operationsActiveMode = false;
      activeOperationalState = standByMode;
      break;
    case noChangeInStatus:
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case standByMode:
      if (operationsActiveMode)
      {
        previousActiveOperationalState = activeOperationalState;
        removeEquipmentOrder();
        activeOperationalState = getNextActiveOperationalState();
      }
      else
      {
        if (reDisplay || (activeOperationalState != previousActiveOperationalState))
        {
          showMessage(lcdMaster, lcdMasterParams, 243, false);
          debugSerialPrint(243, 243, true); //"In standby mode");
          dashboardInfo = 255;
          reDisplay = false;
        }
        if (activeOperationalState != previousActiveOperationalState)
        {
          sendMotorOrdersStop();
          debugSerialPrint(244, 244, false);//"Current Heading: ");
          _SERIAL_PRINTLN(currentHeading);
          sendIntToSlave(dashboardInfo, slaveNodeAddress);
          spreaderLightsOff();
          spreaderIsMoving = false;
          removeEquipmentOrder();
        }
        digitalWrite(steeringMotorPowerSwitch, HIGH);

        previousActiveOperationalState = activeOperationalState;
        checkForMessage();
      }    //
      break;
    case switchedOffMode:
      if (activeOperationalState != previousActiveOperationalState)
      {
        lcdMaster.setCursor(0, 0);
        lcdMaster.print("switched off");

        debugSerialPrint(245, 245, true); //"In Switched Off mode");
        dashboardInfo = 0;
        debugSerialPrint(244, 244, false);//"Current Heading: ");
        _SERIAL_PRINTLN(currentHeading);
        sendIntToSlave(dashboardInfo, slaveNodeAddress);
        spreaderLightsOff();
        spreaderIsMoving = false;
        digitalWrite(steeringMotorPowerSwitch, LOW);
      }
      sendMotorOrdersStop();
      previousActiveOperationalState = activeOperationalState;
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = standByMode;
      break;
    case startDrivingMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("start driving");
      digitalWrite(steeringMotorPowerSwitch, HIGH);

      spreaderIsMoving = false;
      previousActiveOperationalState = activeOperationalState;
      steeringMotorGoStraight(0);
      debugSerialPrint(247, 247, true); // Start Driving
      showMessage(lcdMaster, lcdMasterParams, 247, false);
      logPosition();
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      if (equipmentOrders.size() == 0)
      {
        addEquipmentOrderSegment (straightDrivingMode, 100, 0, 0);
      }
      activeOperationalState = getNextActiveOperationalState();
      lcdMaster.setCursor(0, 1);
      lcdMaster.print("nxt state: ");
      lcdMaster.print(activeOperationalState);
      break;
    case turnLeftMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("turn left");
      showMessage(lcdMaster, lcdMasterParams, 248, false);
      spreaderIsMoving = false;
      debugSerialPrint(248, 248, true); //"In turnLeft");
      previousActiveOperationalState = activeOperationalState;
      leftTurn(currentEquipmentOrder.equipmentOrderModeParameter1, currentEquipmentOrder.equipmentOrderModeParameter2);
      removeEquipmentOrder();

      activeOperationalState = getNextActiveOperationalState();
      break;
    case turnRightMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("turn right");
      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 249, false);
      debugSerialPrint(249, 249, true); //"In turnRight");
      previousActiveOperationalState = activeOperationalState;
      rightTurn(currentEquipmentOrder.equipmentOrderModeParameter1, currentEquipmentOrder.equipmentOrderModeParameter2);
      removeEquipmentOrder();
      activeOperationalState = getNextActiveOperationalState();
      break;
    case straightDrivingMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("straight drive");

      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 191, false);
      debugSerialPrint(91, 91, true); //"Straight Driving");
      if (activeOperationalState != previousActiveOperationalState)
      {
        sendIntToSlave(129, slaveNodeAddress);
      }
      previousActiveOperationalState = activeOperationalState;
      stepsStraight(currentEquipmentOrder.equipmentOrderModeParameter1);
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      checkForMessage();
      logPosition();
      activeOperationalState = getNextActiveOperationalState();
      break;
    case continueDrivingMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("continu drive");

      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 92, false);
      if (activeOperationalState != previousActiveOperationalState)
      {
        debugSerialPrint(92, 92, true); //"In continueDriving");
        sendIntToSlave(195, slaveNodeAddress);
      }
      previousActiveOperationalState = activeOperationalState;
      stepsStraight(100);
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      checkForMessage();
      logPosition();
      if (equipmentOrders.size() == 0)
      {
        addEquipmentOrderSegment (activeOperationalState, 10, 0, 0);
      }
      activeOperationalState = getNextActiveOperationalState();
      break;
    case forwardDrivingMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("forward drive(s)");
      sendMotorOrdersStop();
      spreaderIsMoving = false;
      debugSerialPrint(251, 251, true); //"In switchToForwardDriving");
      setDirectionIndicatorLights();
      previousActiveOperationalState = activeOperationalState;
      setDrivingDirection(drivingForwards);

      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case backwardDrivingMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("back drive(s)");
      sendMotorOrdersStop();
      spreaderIsMoving = false;
      debugSerialPrint(252, 252, true); //"In switchToBackwardDriving");
      setDirectionIndicatorLights();
      //spreaderLightsOn();
      previousActiveOperationalState = activeOperationalState;
      setDrivingDirection(drivingBackwards);
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case stopDrivingMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("stop drive(s)");
      digitalWrite(steeringMotorPowerSwitch, LOW);
      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 253, false);
      sendIntToSlave(255, slaveNodeAddress);
      debugSerialPrint(253, 253, true); //"In stopDriving");
      spreaderLightsOff();
      previousActiveOperationalState = activeOperationalState;
      sendMotorOrdersStop();
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case obstacleAvoidanceMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("obstacle");
      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 254, false);
      debugSerialPrint(254, 254, true); //"In obstacleAvoidance");
      //detachInterrupt(digitalPinToInterrupt(interruptFromSlavePin));
      dashboardInfo = 224;
      sendIntToSlave(dashboardInfo, slaveNodeAddress);
      sendMotorOrdersStop();

      removeAllEquipmentOrders();
      operationsActiveMode = false;
      logPosition();
      dashboardInfo = 128;
      sendIntToSlave(dashboardInfo, slaveNodeAddress);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case checkPositionMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("check position");
      sendMotorOrdersStop();
      previousActiveOperationalState = activeOperationalState;
      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 255, false);
      sendIntToSlave(60, slaveNodeAddress);
      debugSerialPrint(255, 255, true); //"In checkPosition");
      spreaderLightsOff();
      locateVehicle();
      debugSerialPrint(256, 256, true); // "Pos. to ground station");
      logPosition();
      sendCurrentLocationToSerial();
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case rotatePoleMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("rotate radar");
      sendMotorOrdersStop();
      previousActiveOperationalState = activeOperationalState;
      spreaderIsMoving = false;
      showMessage(lcdMaster, lcdMasterParams, 257, false);
      sendIntToSlave(61, slaveNodeAddress);
      debugSerialPrint(257, 257, true); //"In rotatepole");
      spreaderLightsOff();
      if (poleMotorOrientationRelativeToVehicle > 359)
      {
        poleMotorOrientationRelativeToVehicle = poleMotorOrientationRelativeToVehicle - 360;
        if (poleMotorcurrentdirection == anticlockwise)
        {
          poleMotorcurrentdirection = clockwise;
        }
        else
        {
          poleMotorcurrentdirection = anticlockwise;
        }
      }
      poleRequiredrotation = (currentEquipmentOrder.equipmentOrderModeParameter1 << 8) + currentEquipmentOrder.equipmentOrderModeParameter2; // min(30, 360 - poleMotorOrientationRelativeToVehicle);
      if (poleMotorcurrentdirection == anticlockwise)
      {
        poleMotorAntiClockWise(poleRequiredrotation);
      }
      else
      {
        poleMotorClockWise(poleRequiredrotation);
      }
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case liftSpreaderToTopMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("lift spreader");
      sendMotorOrdersStop();
      spreaderIsMoving = true;
      showMessage(lcdMaster, lcdMasterParams, 258, false);
      sendIntToSlave(4, slaveNodeAddress);
      debugSerialPrint(258, 258, true);//"In liftSpreaderToTop:");
      spreaderLightsOn();
      liftSpreader();
      spreaderLightsOff();
      previousActiveOperationalState = activeOperationalState;
      debugSerialPrint(259, 259, false); // Spreader Hoisted

      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();

      spreaderLightsOff();
      spreaderCurrentPosition = spreaderPositionHigh;
      setDirectionIndicatorLights();
      break;
    case dropSpreaderToBottomMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("drop spreader");
      spreaderIsMoving = true;
      showMessage(lcdMaster, lcdMasterParams, 260, false);
      sendMotorOrdersStop();

      sendIntToSlave(64, slaveNodeAddress);
      debugSerialPrint(260, 260, true);//"In dropSpreaderToBottom:");
      spreaderLightsOn();
      dropSpreader();
      spreaderLightsOff();
      previousActiveOperationalState = activeOperationalState;
      debugSerialPrint(261, 261, true); //"Spreader Lowered");
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      spreaderCurrentPosition = spreaderPositionLow;
      setDirectionIndicatorLights();
      break;
    case lockSpreaderMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("lock spreader");
      sendMotorOrdersStop();

      spreaderIsMoving = false;
      sendIntToSlave(8, slaveNodeAddress);
      debugSerialPrint(267, 267, true); //"In lockSpreader");
      digitalWrite(spreaderLockPin, HIGH);
      spreaderOccupied = true;
      setDirectionIndicatorLights();
      spreaderLightsOn();
      previousActiveOperationalState = activeOperationalState;
      debugSerialPrint(99, 99, true); //Spreader locked");
      showMessage(lcdMaster, lcdMasterParams, 98); //"GS:Lock spreader","Spreader locked ");
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case unlockSpreaderMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("unlock spreader");

      sendMotorOrdersStop();
      spreaderIsMoving = false;
      debugSerialPrint(269, 269, true); //"In unlockSpreader");
      sendIntToSlave(32, slaveNodeAddress);
      spreaderLightsOff();
      digitalWrite(spreaderLockPin, LOW);
      spreaderOccupied = false;
      setDirectionIndicatorLights();
      previousActiveOperationalState = activeOperationalState;
      debugSerialPrint(102, 102, true);
      sprintf(logLineBuffer, "Spreader unlocked");
      writeLogLineOnFile(logLineBuffer);
      showMessage(lcdMaster, lcdMasterParams, 101); // "GS:Unlock spr.  ","Spread unlocked ");
      removeEquipmentOrder();
      operationsActiveMode = (equipmentOrders.size() > 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
    case guidedDriveMode:
      lcdMaster.setCursor(0, 0);
      lcdMaster.print("guided drive");

      showMessage(lcdMaster, lcdMasterParams, 270, false);
      sendIntToSlave(41, slaveNodeAddress);
      spreaderIsMoving = false;
      debugSerialPrint(270, 270, true); //"In driveWithGuidance");
      spreaderLightsOn();
      previousActiveOperationalState = activeOperationalState;
      debugSerialPrint(103, 103, true); //"Approach Transfer point");
      sendMotorOrdersStraight (drivingDirection, 0, maxRPM, 0);
      _SERIAL_PRINTLN("main motors on");
      followBlackLinesUntilStop();
      spreaderLightsOff();
      removeEquipmentOrder();
      activeOperationalState = getNextActiveOperationalState();
      break;
    default:
      spreaderIsMoving = false;
      debugSerialPrint(271, 271, false);//"Unknown State: ");
      _SERIAL_PRINTLN(activeOperationalState);
      spreaderLightsOff();
      equipmentOrders.clear();
      operationsActiveMode = false;
      previousActiveOperationalState = activeOperationalState;
      addEquipmentOrderSegment (abortedRunMode, 0, 0, 0);
      activeOperationalState = getNextActiveOperationalState();
      break;
  }
}
