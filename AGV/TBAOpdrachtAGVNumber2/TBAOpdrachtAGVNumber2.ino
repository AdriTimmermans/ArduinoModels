// Pin definitions
//
// Interrupt
//
#define interruptDecoderDiscPin  2 // pin to trigger an interrupt on the master Arduino Nano
//
// Drive motors
//
#define motorAPinB               5 // (PWM) reserved for motor A
#define motorAPinA               4 // (PWM) reserved for motor A
#define motorAEnablePin          6 // (PWM) motorspeed A
#define motorBPinB               7 // (PWM) reserved for motor
#define motorBPinA               8 // (PWM) reserved for motor
#define motorBEnablePin          9 // (PWM) motorspeed B
//
// Steering motors
//
#define steeringAPin            13 // (PWM) servo for steering A
#define steeringBPin            11 // (PWM) servo for steering B
//
// Other pins
//
#define I2CSyncPin               3 // used to synchronise between two masters Wemos or Arduino pro mini, NO PMW-pin will do!
#define IRReceiverPin           A1 // pin to receive IR signals
#define sensor2Y0A21Pin         A2 // Proximity sensor input pin
#define ledSyncPinIsHigh        A3 // used to show when the I2C bus is locked by the nano
//
//
//  SDA                   = A4  // I2C SDA pin
//  SCL                   = A5  // I2C SCL pin
//
#define ledControl              12
//
// definition of buttons on IR remote control unit
//
// button 5
#define __bStop                   0xE0E0906F
#define __bStopAlt                0x731A3E02
//
// button 2
#define __bForwardStraight        0xE0E0A05F
#define __bForwardStraightAlt     0xAD586662
//
// button 1
#define __bForwardLeft            0xE0E020DF
#define __bForwardLeftAlt         0xE13DDA28
//
// button 3
#define __bForwardRight           0x273009C4
#define __bForwardRightAlt        0xCE3693E6
//
// button arrow left
#define __bForwardCrabLeft        0xE0E046B9
#define __bForwardCrabLeftAlt     0x758C9D82
//
// button arrow right
#define __bForwardCrabRight       0xE0E0A659
#define __bForwardCrabRightAlt    0x53801EE8
//
// button 8
#define __bBackwardStraight       0xE0E0B04F
#define __bBackwardStraightAlt    0x6825E53E
//
// button 7
#define __bBackwardLeft           0xE0E030CF
#define __bBackwardLeftAlt        0x4592E14C
//
// button 9
#define __bBackwardRight          0xE0E0708F
#define __bBackwardRightAlt       0x8B8510E8
//
// button spool backwards
#define __bBackwardCrabLeft       0xE0E0A25D
#define __bBackwardCrabLeftAlt    0x75D1D566
//
// button spool forwards
#define __bBackwardCrabRight      0xE0E012ED
#define __bBackwardCrabRightAlt   0x833155A4
//
// button arrow up
#define __bIncreaseSpeed          0xE0E006F9
#define __bIncreaseSpeedAlt       0xC26BF044
//
// button arrow down
#define __bDecreaseSpeed          0xE0E00679
#define __bDecreaseSpeedAlt       0xC4FFB646
//
#define __noButton                0
//
// operational statusses
//
#define __idle                   1
#define __goForwardStraight      2
#define __goForwardLeft          3
#define __goForwardRight         4
#define __goForwardCrabLeft      5
#define __goForwardCrabRight     6
#define __goBackwardStraight     7
#define __goBackwardLeft         8
#define __goBackwardRight        9
#define __goBackwardCrabLeft    10
#define __goBackwardCrabRight   11
#define __stopVehicle           12
#define __increaseSpeed         13
#define __decreaseSpeed         14

enum tofSensorStatusValues
{
  operational,
  failure
};

typedef struct frontAndRearDistanceTemplate frontAndRearDistanceType;

struct frontAndRearDistanceTemplate
{
  int distanceToObjectA;        // distance measured in cm by sharp 2Y0A21 to the front of the AGV
  int distanceToObjectB;         // distance measured in mm by the VL53L01 to the rear of the AGV, converted to cm
};

frontAndRearDistanceType distanceIndicator;

tofSensorStatusValues tofSensorStatus;

//
// IR initialisation
//
#include <IRremote.h>                 // library for IR receiver, to be substituted for wireless communication
#include <Wire.h>                     // library for I2C communication
#include <LiquidCrystal_I2C.h>        // library for LCD usage 
#include <Servo.h>                    // library for using steering (servo)motors
#include <Adafruit_Sensor.h>          // library for use compassmodule (general adafruit library)
#include <Adafruit_HMC5883_U.h>       // library for using the HMC5883 compas module
#include <SharpIR.h>
#include <VL53L0X.h>
//
// Infra red parameters from IR receiver
//
IRrecv irrecv(IRReceiverPin); // create instance of 'irrecv'
decode_results results;       // create instance of results class
//
// vehicle speed control parameters
//
int           vehicleSpeed = 255;
//
// collision detect sensor parameters
//
int           imminentCollision = 0;
bool          makeChoice = true;
bool          lastChoiceWasLeft = false;

//
//  LCD screen
//
const int     i2c_lcdaddr1 = 0x27;
LiquidCrystal_I2C lcdAGV(i2c_lcdaddr1, 20, 4);
//
// steering servos
//
Servo servoA; // front servo
int straightA  = 115;
int leftA      = 140;
int rightA     = 80;
int straightB  = 50;
int leftB      = 25;
int rightB     = 75;
Servo servoB; // rear servo
//
// compass initialisation
//
// Assign a unique ID to this sensor at the same time
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#define addressHMC5883 0x1E //0011110b, I2C 7bit address of HMC5883
byte  modeHMC5883 = 0x02; //mode select register
byte  regHMC5883 = 0x00; //continuous read mode
byte  xregHMC5883 = 0x03; //first data register (1 of 6, MSB and LSB for x, y and z

float uncalibrated_values[3];
float calibrated_values[3];

double calibration_matrix[3][3] =
{
  /*
    {  1.083,  -0.024,  0.024},
    {  0.111,   1.01,   0.617},
    {  0.017,   0.001,  1.049}
  */
  {  1.000,   0.000,  0.000},
  {  0.000,   1.000,  0.000},
  {  0.000,   0.000,  1.000}
};
//hardIronCorrection = bias
//softIronCorrection = matrix above
double hardIronCorrection[3] =
{
  /*  5.109,
    -11.293,
    0.346
  */
  0.000,
  0.000,
  0.000
};
float currentOrientation;
float previousOrientation;
float newOrientation;
float angleCount;
//
// initialize sharp distance sensor
//
#define distanceSensorModel 1080
SharpIR distanceSensor (SharpIR::GP2Y0A21YK0F, sensor2Y0A21Pin);
//
// initialize VL53L0X distance sensor
//
//Adafruit_VL53L0X toFSensor = Adafruit_VL53L0X();
VL53L0X toFSensor;
//
//  enumeration of operational states
//
enum operationalStates
{
  idle                = __idle,
  goForwardStraight   = __goForwardStraight,
  goForwardLeft       = __goForwardLeft,
  goForwardRight      = __goForwardRight,
  goForwardCrabLeft   = __goForwardCrabLeft,
  goForwardCrabRight  = __goForwardCrabRight,
  goBackwardStraight  = __goBackwardStraight,
  goBackwardLeft      = __goBackwardLeft,
  goBackwardRight     = __goBackwardRight,
  goBackwardCrabLeft  = __goBackwardCrabLeft,
  goBackwardCrabRight = __goBackwardCrabRight,
  stopVehicle         = __stopVehicle,
  increaseSpeed       = __increaseSpeed,
  decreaseSpeed       = __decreaseSpeed
};

operationalStates operationalStatus     = idle;
operationalStates lastActiveStatus      = operationalStatus;
operationalStates lastRecognisedStatus  = idle;
operationalStates resumeOperation       = idle;                 // value used to resume previous operation after evasion

long              stopValue              = 0;                    // value used for minimum distance before collision prevention kicks in, determined during startup
int               randomNumber;                                 // value used in randomising behaviour at collision prevention
long              manualTimerStart       = 0;                    // value used for startpoint of manual time-out in the loop-module
bool              newOperation           = false;                // value used to determine when a new action is required
bool              underECS               = false;                // value to determine if the AGV is operating automatically or under ECS
volatile int      decoderDiscCount       = 0;                    // value used to determine distance traveled
float             wheelDiameter          = 3.40;
int               pulsesPerWheelRotation = 20;
float             distancePerPulse       = 2.0 * M_PI * wheelDiameter / pulsesPerWheelRotation;
bool              testMode               = false;

enum evasiveAction
{
  noAction = 0,
  backup   = 1,
  turn     = 2
};

evasiveAction evasionInProgress = noAction;
float                                                            = 0.0;                    // value used for the required distance to back up after evasive action
long              distancePollingTimer  = 0;                    // value used for polling the distance sensor

void countDecoderDisc()
{
  decoderDiscCount++;
}

float distanceTraveled()
{
  //
  // function to calculate the distance traveled since last change of operation
  //
  float aux;
  //
  aux = decoderDiscCount * distancePerPulse;
  //
  return aux;
}

bool claimI2CBus()
{
  //
  // this module claims the I2C bus before using in case of multi-master setup
  //
  bool aux = false;
  long startClaimTimeOut = millis();
  int  lineStatus;
  bool continuePolling = true;
  while (continuePolling)
  {
    lineStatus = digitalRead(I2CSyncPin);
    if (lineStatus == HIGH)
    {
      continuePolling = false;
      aux = true;
      pinMode(I2CSyncPin, OUTPUT);
      digitalWrite(I2CSyncPin, LOW);
    }
    else
    {
      delay(random(10) + 50);
      continuePolling = ((millis() - startClaimTimeOut) < 500);
    }
  }

  return aux;
}

void releaseI2CBus()
{
  //
  // used for accomodating multi-master I2C setup
  // After using the I2C bus by this MC, the line is set to default HIGH again and set to INPUT
  //
  pinMode(I2CSyncPin, INPUT);
  delay(50);
}

void transformation(float x, float y, float z)
{
  //
  // Module to transform the hard- and soft-iron  correction on the compass reading
  //
  uncalibrated_values[0] = x;
  uncalibrated_values[1] = y;
  uncalibrated_values[2] = z;
  //
  //calculation
  //
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * (uncalibrated_values[j] - hardIronCorrection[j]);
  for (int i = 0; i < 3; ++i) calibrated_values[i] = result[i];
}

float getHeading()
{
  //
  // this module returns the heading as measured by the HMC5883 sensor
  //

  float mGaussx;
  float mGaussy;
  float mGaussz;

  float headingDegrees = 0;

  if (claimI2CBus())
  {
    Wire.beginTransmission(addressHMC5883); //open communication with HMC5883
    Wire.write(modeHMC5883); //select mode register
    Wire.write(regHMC5883); //continuous measurement mode
    Wire.endTransmission();

    sensors_event_t event;
    mag.getEvent(&event);
    mGaussx = event.magnetic.x;
    mGaussy = event.magnetic.y;
    mGaussz = event.magnetic.z;
    releaseI2CBus();
    /*
      _SERIAL_PRINT("X: "); _SERIAL_PRINT(event.magnetic.x); _SERIAL_PRINT("  ");
      _SERIAL_PRINT("Y: "); _SERIAL_PRINT(event.magnetic.y); _SERIAL_PRINT("  ");
      _SERIAL_PRINT("Z: "); _SERIAL_PRINT(event.magnetic.z); _SERIAL_PRINTLN(" uT ");
    */
    transformation(mGaussx, mGaussy, mGaussz);
    //
    float headingxy = atan2(calibrated_values[0], calibrated_values[1]);
    //
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    //
    float declinationAngle = 0.021;
    headingxy += declinationAngle;
    //
    // Correct for when signs are reversed.
    //
    if (headingxy < 0)
      headingxy += 2 * M_PI;
    //
    // Check for wrap due to addition of declination.
    //
    if (headingxy > 2 * M_PI)
      headingxy -= 2 * M_PI;
    //
    // Convert radians to degrees for readability.
    //
    headingDegrees = 360.0 - headingxy * 180 / M_PI;

  }
  return headingDegrees;
}

void motorConfig (int angleA, int angleB, int pinAA, int pinBA, int _vehicleSpeed, String lcdText)
{
  //
  // module to limit the number of lines in the motorSetting module
  //
  // display first
  //
  displayLCD (lcdText, _vehicleSpeed, 3);
  //
  // turn steering wheels
  //
  servoB.write(angleA);
  servoA.write(angleB);
  //
  // set driving direction
  //
  if (_vehicleSpeed == 0)
  {
    digitalWrite(motorAPinA, LOW);
    digitalWrite(motorAPinB, LOW);
    digitalWrite(motorBPinA, LOW);
    digitalWrite(motorBPinB, LOW);
  }
  else
  {
    digitalWrite(motorAPinA, pinAA);
    (pinAA == HIGH) ? digitalWrite(motorAPinB, LOW) : digitalWrite(motorAPinB, HIGH);
    digitalWrite(motorBPinA, pinBA);
    (pinBA == HIGH) ? digitalWrite(motorBPinB, LOW) : digitalWrite(motorBPinB, HIGH);
  }
  //
  // start engines
  //
  analogWrite(motorAEnablePin, _vehicleSpeed);
  analogWrite(motorBEnablePin, _vehicleSpeed);
}

void displayLCD (String line2, int parameterValue, byte lineNr)
{
  //
  // module to display the second line of the text on the LCD screen, including the overall vehicle speed between 0 and 255
  //
  if (claimI2CBus())
  {
    digitalWrite(ledSyncPinIsHigh, HIGH);
    lcdAGV.setCursor(0, lineNr);
    lcdAGV.print("                    ");
    lcdAGV.setCursor(0, lineNr);
    lcdAGV.print(line2);
    lcdAGV.setCursor(16, lineNr);
    lcdAGV.print(parameterValue);
    releaseI2CBus();
    digitalWrite(ledSyncPinIsHigh, LOW);
  }
}

void displayLCD2Values (int par1, int par2, byte lineNr)
{
  //
  // module to display two values on the LCD screen, including the overall vehicle speed between 0 and 255
  //
  if (claimI2CBus())
  {
    digitalWrite(ledSyncPinIsHigh, HIGH);
    lcdAGV.setCursor(0, lineNr);
    lcdAGV.print("                    ");
    lcdAGV.setCursor(0, lineNr);
    lcdAGV.print(par1);
    lcdAGV.setCursor(11, lineNr);
    lcdAGV.print(par2);
    releaseI2CBus();
    digitalWrite(ledSyncPinIsHigh, LOW);
  }
}

void moveVehicle (operationalStates state, int _vehicleSpeed)
{
  //
  // module to define the movement of the vehicle for forward/backward, turning and crab-moving
  //
  // set motorconfiguration of both  motors to go forward, start the motors and do nothing until another status is received
  // either by the remote control or by the collision prevention algoritm
  //
  Serial.print("S ");
  Serial.print(state);
  switch (state)
  {
    case __goForwardStraight:
      motorConfig (straightA, straightB, HIGH, HIGH, _vehicleSpeed, "Forward");
      Serial.print("F");
      break;
    case __goForwardLeft:
      motorConfig (leftA, leftB, HIGH, HIGH, _vehicleSpeed, "Forward left");
      Serial.print("FL");
      break;
    case __goForwardRight:
      motorConfig (rightA, rightB, HIGH, HIGH, _vehicleSpeed, "Forward right");
      Serial.print("FR");
      break;
    case __goForwardCrabLeft:
      motorConfig (leftA, rightB, HIGH, HIGH, _vehicleSpeed, "Forward crab L");
      Serial.print("FCL");
      break;
    case __goForwardCrabRight:
      motorConfig (rightA, leftB, HIGH, HIGH, _vehicleSpeed, "Forward crab R");
      Serial.print("FCR");
      break;
    case __goBackwardStraight:
      motorConfig (straightA, straightB, LOW, LOW, _vehicleSpeed, "Backward");
      Serial.print("B");
      break;
    case __goBackwardLeft:
      motorConfig (leftA, leftB, LOW, LOW, _vehicleSpeed, "Backward left");
      Serial.print("BL");
      break;
    case __goBackwardRight:
      motorConfig (rightA, rightB, LOW, LOW, _vehicleSpeed, "Backward right");
      Serial.print("BR");
      break;
    case __goBackwardCrabLeft:
      motorConfig (rightA, leftB, LOW, LOW, _vehicleSpeed, "Backward crab L");
      Serial.print("BCL");
      break;
    case __goBackwardCrabRight:
      motorConfig (leftA, rightB, LOW, LOW, _vehicleSpeed, "Backward crab R");
      Serial.print("BCR");
      break;
    case __idle:
    case __stopVehicle:
      motorConfig (straightA, straightB, LOW, LOW, 0, "Stop");
      Serial.print("S");
      break;
  }
  Serial.println("");
}

void irReceive()
{
  //
  // module to receive and decode the message from the master and translate the Remote Control button to a operational state
  //
  if (irrecv.decode(&results))
  {
    operationalStatus     = translateIR(results.value);
    lastRecognisedStatus  = operationalStatus;
    newOperation          = true;
    digitalWrite(ledControl, LOW);
    evasionInProgress     = noAction;
    backUpDistance        = 0.0;     // value used for the time required to back up after evasive action
    decoderDiscCount      = 0;
    irrecv.resume() ;
  }
}

operationalStates translateIR(long codeToTranslate)
{
  //
  // module to translate the button pressed on the remote control into a cammand on the AGV
  //
  operationalStates aux = lastRecognisedStatus;
  // translate a Remote control button press into an operational state, it does not matter if it comes from the Slave (front of the vehicle) or the Master (back of the vehicle)
  newOperation = true;
  switch (codeToTranslate)
  {
    case __bStop:
    case __bStopAlt:
      aux = stopVehicle;
      break;
    case __bForwardStraight:
    case __bForwardStraightAlt:
      aux = goForwardStraight;
      break;
    case __bForwardLeft:
    case __bForwardLeftAlt:
      aux = goForwardLeft;
      break;
    case __bForwardRight:
    case __bForwardRightAlt:
      aux = goForwardRight;
      break;
    case __bForwardCrabLeft:
    case __bForwardCrabLeftAlt:
      aux = goForwardCrabLeft;
      break;
    case __bForwardCrabRight:
    case __bForwardCrabRightAlt:
      aux = goForwardCrabRight;
      break;
    case __bBackwardStraight:
    case __bBackwardStraightAlt:
      aux = goBackwardStraight;
      break;
    case __bBackwardLeft:
    case __bBackwardLeftAlt:
      aux = goBackwardLeft;
      break;
    case __bBackwardRight:
    case __bBackwardRightAlt:
      aux = goBackwardRight;
      break;
    case __bBackwardCrabLeft:
    case __bBackwardCrabLeftAlt:
      aux = goBackwardCrabLeft;
      break;
    case __bBackwardCrabRight:
    case __bBackwardCrabRightAlt:
      aux = goBackwardCrabRight;
      break;
    case __bIncreaseSpeed:
    case __bIncreaseSpeedAlt:
      aux = increaseSpeed;
      break;
    case __bDecreaseSpeed:
    case __bDecreaseSpeedAlt:
      aux = decreaseSpeed;
      break;

    default:
      aux = lastRecognisedStatus;
      newOperation = false;
  }

  return aux;
}

void displayOrientation()
{
  //
  // Module to display the orientation at regular intervals
  //
  float displayOrientation = getHeading();
  displayLCD("Orientation:", (int)displayOrientation, 0);
}

void shutdownVehicle ()
{
  //
  // Module to stop the vehicle and set the steering to straight
  //
  moveVehicle (stopVehicle, 0);
  operationalStatus = idle;
}

operationalStates chooseMovementOption(operationalStates currentOption)
{
  //
  // module to switch driving under auto driving
  //
  operationalStates aux = currentOption;
  if (!underECS)
  {
    byte movement = random(10) + 1;
    switch (movement)
    {
      case 7:
      case 8:
      case 1:
        aux = goForwardStraight;
        break;
      case 9:
      case 10:
      case 2:
        aux = goForwardStraight;
        break;
      case 3:
        aux = goForwardCrabLeft;
        break;
      case 4:
        aux = goForwardCrabRight;
        break;
      case 5:
        aux = goBackwardCrabLeft;
        break;
      case 6:
        aux = goBackwardCrabRight;
        break;
      default:
        aux = goForwardStraight;
        break;
    }
  }
  return aux;
}

void checkTimer()
{
  //
  // module to manually set a timer, TimerOne.h does not function correctly
  //
  if ((millis() - manualTimerStart) > 5000)
  {
    displayOrientation();
    manualTimerStart = millis();
    float checkDistance = distanceTraveled();
    Serial.print("decoderDiscCount: ");
    Serial.print(decoderDiscCount);
    Serial.print("distance: ");
    Serial.println(checkDistance);
    if (evasionInProgress == noAction)
    {
      if (checkDistance > 400.0)
      {
        operationalStatus = chooseMovementOption(operationalStatus);
        displayLCD("Distance :", checkDistance, 2);
        displayLCD("Pulses :", decoderDiscCount, 3);
        shutdownVehicle();
        delay(5000);
        newOperation = true;
        decoderDiscCount = 0;
      }
    }
  }
}

int reliableDistance()
{
  //
  // module to find the distance to an object to the rear of the AGV
  //
  //VL53L0X_RangingMeasurementData_t measure;
  int distanceInMM;

  distanceInMM = toFSensor.readRangeSingleMillimeters();

  if (toFSensor.timeoutOccurred())
  {
    distanceInMM = -1;
  }
  //
  //

  return distanceInMM;
}
bool withinRange (int checkValue, int range)
{
  //
  // this module checks if the value is >0 and <= range)
  //
  return ((checkValue <= range) && (checkValue > 0));
}

frontAndRearDistanceType pollingDistance (int lastKnownDistanceA, int lastKnownDistanceB)
{
  //
  // this module checks the distance every 200 miliseconds to prevent false positives
  //
  frontAndRearDistanceType aux;
  frontAndRearDistanceType auxCheck1;
  frontAndRearDistanceType auxCheck2;
  //
  // initialize return values
  //
  aux.distanceToObjectA = lastKnownDistanceA;
  aux.distanceToObjectB  = lastKnownDistanceB;
  //
  if ((millis() - distancePollingTimer) > 200)
  {
    distancePollingTimer = millis();
    //
    // check distance to the front
    //
    aux.distanceToObjectA = reliableDistance() / 10;
    if (withinRange(aux.distanceToObjectA, stopValue))
    {
      //
      // check for false positives, 3 times the measuring within range = OK
      //
      delay(50);
      auxCheck1.distanceToObjectA = reliableDistance() / 10;;
      delay(50);
      auxCheck2.distanceToObjectA = reliableDistance() / 10;;

      if (!(withinRange(auxCheck1.distanceToObjectA, stopValue) && withinRange(auxCheck2.distanceToObjectA, stopValue)))
      {
        //
        // distance of one measurement was not within range: this must be a false positive
        //
        aux.distanceToObjectA = lastKnownDistanceA;
      }
    }
    else
    {
      lastKnownDistanceA = aux.distanceToObjectA;
    }
    //
    // measure distance from the rear of the vehicle
    //
    aux.distanceToObjectB = distanceSensor.getDistance();
    if (withinRange(aux.distanceToObjectB, stopValue))
    {
      //
      // check for false positives by taking 2 more measurements, if all are within range:
      // it must be correct
      //
      delay(50);
      auxCheck1.distanceToObjectB = distanceSensor.getDistance();
      delay(50);
      auxCheck2.distanceToObjectB = distanceSensor.getDistance();

      if (!(withinRange(auxCheck1.distanceToObjectB, stopValue) && withinRange(auxCheck2.distanceToObjectB, stopValue)))
      {
        //
        // distance of one measurement was not within range: this must be a false positive
        //
        aux.distanceToObjectB = lastKnownDistanceB;
      }
    }
    else
    {
      lastKnownDistanceB = aux.distanceToObjectB;
    }
  }
  //
  return aux;
}

int motionDirection ()
{
  //
  // module to determine if the operational status is moving forward, backwards or standing still
  //
  int aux;
  switch (operationalStatus)
  {
    case idle:
    case stopVehicle:
      aux = 0;
      break;
    case goForwardStraight:
    case goForwardCrabRight:
    case goForwardRight:
    case goForwardLeft:
    case goForwardCrabLeft:
      aux = 1;
      break;
    case goBackwardRight:
    case goBackwardLeft:
    case goBackwardCrabLeft:
    case goBackwardCrabRight:
    case goBackwardStraight:
      aux = -1;
      break;
    default:
      aux = 0;
      break;
  }
  return aux;
}

bool turnCompleted()
{
  //
  // module to calculate if the turn was completed
  //
  float angleDifference;
  bool aux = false;

  currentOrientation = getHeading();
  Serial.print("Previous orientation and current orientation: ");
  Serial.print(previousOrientation);
  Serial.print("\t");
  Serial.println(currentOrientation);
  angleDifference = previousOrientation -  currentOrientation;
  if (abs(angleDifference) > 180.0)
  {
    (angleDifference < 0) ? angleDifference = angleDifference + 360.0 : angleDifference = angleDifference - 360.0;
  }
  angleCount = angleCount + angleDifference;

  aux = (abs(angleCount) > 90.0);

  Serial.print("Started turn: ");
  Serial.println(angleCount);
  previousOrientation = currentOrientation;
  //
  return aux;
}

void setup()
{
  int aux;
  //
  // Setup module, this module is executed only once after startup of the arduino
  //
  Serial.begin(9600);
  //
  // set all the control pins
  //
  pinMode(motorAEnablePin,          OUTPUT);
  pinMode(motorBEnablePin,          OUTPUT);
  pinMode(motorAPinA,               OUTPUT);
  pinMode(motorAPinB,               OUTPUT);
  pinMode(motorBPinA,               OUTPUT);
  pinMode(motorBPinB,               OUTPUT);
  pinMode(steeringAPin,             OUTPUT);
  pinMode(steeringBPin,             OUTPUT);
  pinMode(I2CSyncPin,               INPUT);
  pinMode(IRReceiverPin,            INPUT);
  pinMode(interruptDecoderDiscPin,  INPUT_PULLUP);
  pinMode(ledControl,               OUTPUT);
  digitalWrite(ledControl, LOW);
  //
  // initialise steering servos
  //
  servoA.attach(steeringBPin);  // attaches the servo on pin 11 to the servo object
  servoA.write(straightB);
  servoB.attach(steeringAPin);  // attaches the servo on pin 13 to the servo object
  servoB.write(straightA);
  //
  // set up I2C bus (using 2 masters)
  //
  Wire.begin();
  //
  // initialize LCD for using with both masters
  //

  while (!claimI2CBus())
  {};
  Serial.println("Bus is claimed");
  digitalWrite(ledSyncPinIsHigh, HIGH);
  lcdAGV.begin(20, 4);
  lcdAGV.setBacklight(255);
  lcdAGV.setCursor(0, 0);
  releaseI2CBus();
  digitalWrite(ledSyncPinIsHigh, LOW);
  displayLCD ("Starting up ", 0, 3);
  delay(2000);
  for (int i = 1; i < 6; i++)
  {
    displayLCD ("Go at 5: ", i, 3);
    delay(1000);
  }
  //
  // take the average distance (over 10 samples) to the object in front of the vehicle to be the closest distance before starting collision prevention actions
  //
  for (int i = 1; i <= 10; i++)
  {
    displayLCD ("Objects(10): ", i, 3);
    aux = distanceSensor.getDistance();
    stopValue = stopValue + aux;
    Serial.print("Sv ");
    Serial.println(aux);
    delay(400);
  }
  stopValue = min(stopValue / 10, 20);
  displayLCD ("Object at ", stopValue, 3);
  distanceIndicator.distanceToObjectB = distanceSensor.getDistance();
  //
  // if vehicle not in front of an obstacle, obstacle detection will be effectively off
  //
  stopValue = min(30, stopValue);
  delay(1000);

  moveVehicle(goBackwardStraight, vehicleSpeed); // move the vehicle back and shutdown to wait for the next command
  delay(2000);
  shutdownVehicle();
  //
  // Initialise IR
  //
  irrecv.enableIRIn();
  //
  // initialise 5 second update for display of the orientation
  //
  if (toFSensor.init())
  {
    toFSensor.setMeasurementTimingBudget(200000);


    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.

    bool measurementReliable = reliableDistance();
    tofSensorStatus = operational;
  }
  else
  {
    tofSensorStatus = failure;
  }

  distanceIndicator.distanceToObjectA = reliableDistance() / 10;
  //
  // get current orientation
  //
  currentOrientation = getHeading();
  //
  //  connect interrupt to interrupt function
  //
  attachInterrupt(digitalPinToInterrupt(interruptDecoderDiscPin), countDecoderDisc, RISING);

}

void loop()
{
  // in the main loop: execute each command individually until a new command is entered or generated
  checkTimer();
  if (newOperation)
  {
    switch (operationalStatus)
    {
      case idle:
        lastActiveStatus = operationalStatus;
        newOperation = false;
        break;
      case goForwardStraight:
      case goBackwardStraight:
      case goForwardRight:
      case goBackwardRight:
      case goForwardLeft:
      case goBackwardLeft:
        moveVehicle (operationalStatus, vehicleSpeed);
        lastActiveStatus = operationalStatus;
        newOperation = false;
        break;
      case goForwardCrabLeft:
      case goBackwardCrabLeft:
      case goForwardCrabRight:
      case goBackwardCrabRight:
        moveVehicle (operationalStatus, vehicleSpeed);
        lastActiveStatus = operationalStatus;
        newOperation = false;
        break;
      case stopVehicle:
        shutdownVehicle();
        lastActiveStatus = operationalStatus;
        operationalStatus = idle;
        newOperation = true;
        break;
      case increaseSpeed:
        vehicleSpeed = min(vehicleSpeed + 5, 255);
        operationalStatus = lastActiveStatus;
        newOperation = true;
        break;
      case decreaseSpeed:
        vehicleSpeed = max(vehicleSpeed - 5, 100);
        operationalStatus = lastActiveStatus;
        newOperation = true;
        break;
      default:
        operationalStatus = lastActiveStatus;
        newOperation = true;
        break;
    }
  }
  //
  // after excuting the command, check if there are new IR commands in the IR-buffer of the master computer
  //

  irReceive();
  //
  // and check the distance to the nearest object in front
  //
  distanceIndicator = pollingDistance(distanceIndicator.distanceToObjectA, distanceIndicator.distanceToObjectB);
  displayLCD2Values (distanceIndicator.distanceToObjectA, distanceIndicator.distanceToObjectB, 1);
  if (withinRange(distanceIndicator.distanceToObjectA, stopValue) && (motionDirection() == 1))
  {
    if (evasionInProgress == noAction)
    {
      Serial.println("Object detected driving forwards");
      resumeOperation = operationalStatus;
      evasionInProgress = backup;
      digitalWrite(ledControl, HIGH);
      operationalStatus = goBackwardStraight;
      decoderDiscCount = 0;
      newOperation = true;
      displayLCD ("Object at A ", distanceIndicator.distanceToObjectA, 2);
    }
    else
    {
      resumeOperation = operationalStatus;
      moveVehicle (stopVehicle, 0);
      for (int i = 1; i < 21; i++)
      {
        displayLCD ("Wait until 20:", i, 2);
        delay(1000);
      }
      operationalStatus = resumeOperation;
      moveVehicle (operationalStatus, vehicleSpeed);
    }
  }
  else if (withinRange(distanceIndicator.distanceToObjectB, stopValue) && (motionDirection() == -1))
  {
    if (evasionInProgress == noAction)
    {
      Serial.println("Object detected driving backwards");
      resumeOperation = operationalStatus;
      evasionInProgress = backup;
      digitalWrite(ledControl, HIGH);
      operationalStatus = goForwardStraight;
      decoderDiscCount = 0;
      newOperation = true;
      displayLCD ("Object at rear", distanceIndicator.distanceToObjectB, 2);
    }
    else
    {
      resumeOperation = operationalStatus;
      moveVehicle (stopVehicle, 0);
      for (int i = 1; i < 21; i++)
      {
        displayLCD ("Wait until 20:", i, 2);
        delay(1000);
      }
      operationalStatus = resumeOperation;
      moveVehicle (operationalStatus, vehicleSpeed);
    }
  }
  //
  //  in case of evasive action the switch will handle the actions.
  //
  switch (evasionInProgress)
  {
    case backup:
      backUpDistance = distanceTraveled();
      if (backUpDistance >= 600.0)
      {
        evasionInProgress = turn;
        previousOrientation = currentOrientation;
        angleCount = 0.0;
        if (makeChoice)
        {
          randomNumber = random(100);
          if (randomNumber > 50)
          {
            if (operationalStatus == goBackwardStraight)
            {
              operationalStatus = goForwardRight;
              newOrientation = currentOrientation + 90.0;
            }
            else
            {
              operationalStatus = goBackwardRight;
              newOrientation = currentOrientation - 90.0;
            }
            lastChoiceWasLeft = false;
          }
          else
          {
            if (operationalStatus == goBackwardStraight)
            {
              operationalStatus = goForwardLeft;
              newOrientation = currentOrientation - 90.0;
            }
            else
            {
              operationalStatus = goBackwardLeft;
              newOrientation = currentOrientation + 90.0;
            }
            lastChoiceWasLeft = true;
          }
          makeChoice = false;
        }
        else
        {
          //
          // deflect twice to the same side to prevent being caught in a corner
          //
          if (operationalStatus == goBackwardStraight)
          {
            if (lastChoiceWasLeft) {
              operationalStatus = goForwardLeft;
              newOrientation = currentOrientation - 90.0;
            }
            else
            {
              operationalStatus = goForwardRight;
              newOrientation = currentOrientation + 90.0;
            }
          }
          else
          {
            if (lastChoiceWasLeft)
            {
              operationalStatus = goBackwardLeft;
              newOrientation = currentOrientation + 90.0;
            }
            else
            {
              operationalStatus = goBackwardRight;
              newOrientation = currentOrientation - 90.0;
            }
          }
          makeChoice = true;
        }
        newOperation = true;
      }
      break;
    case turn:
      if (turnCompleted())
      {
        digitalWrite(ledControl, LOW);
        evasionInProgress = noAction;
        operationalStatus = resumeOperation;
        newOperation = true;
        decoderDiscCount = 0;
      }
      break;
  }

}
