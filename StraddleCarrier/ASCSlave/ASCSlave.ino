// This is the slave on the A-strad with id=9

#define BUTTON_INPUT_REQUIRED 1

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <ASCCommunicationDefinition.h>
#include <Adafruit_NeoPixel.h>
#include <TimerOne.h>

const float motorCorrection = 0.80;
// pindefinitions:
#define interruptSpreaderMotorCountPin  2
#define NEOPin                          5    // control for NEO-Pixel
#define spreaderTouchPin                7    // spreaderSwitchPin
#define masterReadyForSlavePin         10    // signal to see if slave should signal ready or just continue
#define interruptPin                   11    // trigger interrupt collision detect -> pin 2 on master
#define interruptSpreaderPin           12    // trigger spreader height -> pin 3 on master
//      I2C SCL                        20
//      I2C SDA                        21
#define receiveDataLEDPin              26
#define requestDataLEDPin              25
#define SROSpreaderTriggerPin          34
#define SROSpreaderEchoPin             35
#define SROTriggerPin                  36
#define SROEchoPin                     37
#define SDCardChipSelectPin            53    // pin to enable the SDcard 
//      SPI MOSI                       50
//      SPI MISO                       51
//      SPI CLK                        52

volatile bool receiveEventCalled = false;
volatile bool requestEventCalled = false;

int distance;
bool          OperationalModeActive = true; // no collision detect actions when in testmode (no tinterrupt send to main computer

int           falsePositiveCount = 0;
int           spreaderHeight;  // in mm
volatile int  spreaderCount = 0;
volatile int  requiredSpreaderCount = 0;
double        auxCalculate;
const double  mmPerRotation = 70.00;
int           spreaderMoveDistance = 0;
bool          spreaderSuspended = false;
int           numberOfPulsesGoingUp = 0;
volatile byte interrupt11Allowed = true;
byte          lineNumber;
volatile int  requiredSpreaderPosition;

char lineOne[80];
char lineTwo[80];


enum spreaderStates
{
  atTop       = 0,
  inMiddle    = 1,
  atBottom    = 2
};

spreaderStates currentSpreaderState;

// Define ASC Nrs

#define ASCnotIdentified 0
#define ASCnr1 1
#define ASCnr2 2

#define numberOfLEDs 8

// LED numbers in NEOPixel for leds. replaced by NEOPixel

#define spreaderDetectLED 0
#define spreaderDetectBit 1

#define moveLeftLED 1
#define moveLeftBit 2

#define moveForwardLED 2
#define moveForwardBit 4

#define waitingFororderLED 3
#define waitingFororderBit 8

#define moveBackwardLED 4
#define moveBackwardBit 16

#define moveRightLED 5
#define moveRightBit 32

#define colisionDetectLED  6
#define colisionDetectBit 64

#define readyForinputLED  7
#define readyForinputBit 128

// define colors

uint32_t WHITE  = 0x00FFFFFF;
uint32_t RED    = 0x00FF0000;
uint32_t GREEN  = 0x0000FF00;
uint32_t BLUE   = 0x000000FF;
uint32_t ORANGE = 0x00FFAF00;
uint32_t YELLOW = 0x00FFFF00;

//const char* displayStrings[] = {"Spreader--------", "Left------------", "Forward---------", "Waiting---------", "Backward--------", "Right-----------", "Collision detect", "-RFI", "----------------"};
const byte ledValues [] = {1, 2, 4, 8, 16, 32, 64, 128};
//    const uint32_t NEOColors [] = {RED, ORANGE, YELLOW, GREEN, YELLOW, ORANGE, RED, BLUE};
const uint32_t NEOColors [] = {WHITE,             YELLOW,            GREEN,            ORANGE,            RED,                BLUE,               RED,                WHITE,  WHITE};
// Create new LCD instance

// dashboardInfo

volatile int dashboardInfo = 0;
volatile boolean statusChanged;
volatile byte nextEvent = 0;
#define unknown 0
#define dashboardInfoData  1
#define logLineInfoAnnouncement  2

float previousorientation;
boolean orientationstatusChanged;
int nearestObjectDistance;
volatile int masterCommand = 0;
// Identify Straddle

int ASCnr = ASCnotIdentified;

// create NEOPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numberOfLEDs, NEOPin, NEO_GRB + NEO_KHZ800);

long startTesttime;
long testTimeinmiliseconds;
int masterInfoRequest = 0;

#define NODE_ADDRESS 0x09

volatile byte byteSlaveAnswer;
volatile byte slaveReadyStatus = slaveBusy;
volatile bool heartStillBeating = true;

int normalStartOrRecover = 0; // 0 = normal start, 1 = recover from hanging I2C


volatile byte interruptType = noMessage;

void handleSpreaderMotorSensor()
{
  spreaderCount++;
  _SERIAL_PRINT("Spreader counter:");
  _SERIAL_PRINTLN(spreaderCount);
}

void triggerRequestInterruptType (byte _interruptType)
{
  if (interrupt11Allowed)
  {
    interruptType = _interruptType;
    triggerInterruptLocal(interruptPin);
  }
  else
  {
    _SERIAL_PRINTLN("No interrupt allowed at this time");
  }
}

void(* resetFunc) (void) = 0;//declare reset function at address 0

int findSpreaderHeight()
{

  bool validValueFound = false;
  long distanceInMM;
  long du;
  int checkCount = 0;

  do
  {
    digitalWrite(SROSpreaderTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(SROSpreaderTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(SROSpreaderTriggerPin, LOW);

    du = pulseIn(SROSpreaderEchoPin, HIGH);
    if (du > 1484)
    {
      distanceInMM = 0;
    }
    else
    {
      distanceInMM = du / 5.82;
      validValueFound = (distanceInMM > 35) && (distanceInMM < 300);
    }
    checkCount++;
    _SERIAL_PRINT("Distance count: ");
    _SERIAL_PRINT(checkCount);
    _SERIAL_PRINT(", Distance: ");
    _SERIAL_PRINTLN(distanceInMM);
    if (checkCount > 10)
    {
      distanceInMM = 140;
      validValueFound = true;
    }
  }
  while (!validValueFound);

  if (distanceInMM < 60)
  {
    currentSpreaderState = atTop;
  }
  else if (distanceInMM > 200)
  {
    currentSpreaderState =  atBottom;
  }
  else
  {
    currentSpreaderState = inMiddle;
  }
  return distanceInMM;
}

void testNEOPixel()
{
  colorWipe(strip.Color(255,   0,   0), 50); // Red
  colorWipe(strip.Color(  0, 255,   0), 50); // Green
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  // Do a theater marquee effect in various colors...
  theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  rainbow(2);             // Flowing rainbow cycle along the whole strip
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
  colorWipe(strip.Color(0,   0,   0), 50); // off
}

void colorWipe(uint32_t color, int wait)
{
  for (int i = 0; i < strip.numPixels(); i++)
  { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(2);                           //  Pause for a moment
  }
}

void rainbow(int wait) {

  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for (int a = 0; a < 30; a++) { // Repeat 30 times...
    for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
void theaterChase(uint32_t color, int wait) {
  for (int a = 0; a < 10; a++) { // Repeat 10 times...
    for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

int findDistance() {

  long di, du;

  digitalWrite(SROTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(SROTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(SROTriggerPin, LOW);

  du = pulseIn(SROEchoPin, HIGH);
  // du > 4656 means > 80 cm which is out of range
  du = min(4656, max(0, du));
  (du > 4656) ? di = 80 : di = du / 58.2;
  return di;
}

void triggerInterruptLocal (int intPin)
{
  testTimeinmiliseconds = millis() - startTesttime; // do not trigger other arduino in first 5 seconds of the test.

  if (testTimeinmiliseconds > 5000) {
    digitalWrite (intPin, HIGH); //
    delay(2); // 2 ms pulse
    digitalWrite (intPin, LOW); //

    _SERIAL_PRINT("Trigger interrupt on pin nr:");
    _SERIAL_PRINTLN(intPin);
  }
}

void lightXmasTree()
{
  int labels = 0;
  //
  // show leds
  //
  _SERIAL_PRINT("NEOstrip input:");
  _SERIAL_PRINTLN(dashboardInfo);
  setDashboardLED (dashboardInfo, waitingFororderBit, waitingFororderLED);
  setDashboardLED (dashboardInfo, moveForwardBit, moveForwardLED);
  setDashboardLED (dashboardInfo, moveLeftBit, moveLeftLED);
  setDashboardLED (dashboardInfo, moveRightBit, moveRightLED);
  setDashboardLED (dashboardInfo, moveBackwardBit, moveBackwardLED);
  setDashboardLED (dashboardInfo, colisionDetectBit, colisionDetectLED);
  setDashboardLED (dashboardInfo, spreaderDetectBit, spreaderDetectLED);
  setDashboardLED (dashboardInfo, readyForinputBit, readyForinputLED);
  delay(10);
}

void receiveEvent(int howMany)
{
  int aux;
  int characterCount;
  _SERIAL_PRINTLN("I2C receive coming in");
  digitalWrite(receiveDataLEDPin, HIGH);
  receiveEventCalled = true;
  if (nextEvent == unknown)
  {
    if (howMany == 1)
    {
      aux = Wire.read();
      _SERIAL_PRINT("Message received: ");
      _SERIAL_PRINTLN(aux);
      switch (aux)
      {
        case requestToLiftSpreader:    // checked
          interrupt11Allowed = true;
          requiredSpreaderPosition = 50;
          if (findSpreaderHeight() < 70)
          {
            _SERIAL_PRINTLN("Spreader already in top");
            requiredSpreaderPosition = 0;
          }
          nextEvent = unknown;
          break;
        case requestToDropSpreader:    //checked
          interrupt11Allowed = true;
          requiredSpreaderPosition = 162 ;
          if (findSpreaderHeight() > 180)
          {
            _SERIAL_PRINTLN("Spreader already at bottom");
            requiredSpreaderPosition = 0;
          }
          nextEvent = unknown;
          break;
        case informToReceiveDashboardInfo:
          interrupt11Allowed = false;
          masterCommand = aux;
          nextEvent = dashboardInfoData;
          break;
        case requestInterruptType:
          interrupt11Allowed = false;
          masterCommand = aux;
          nextEvent = unknown;
          break;
        case requestSpreaderHeight:
          interrupt11Allowed = false;
          masterCommand = aux;
          nextEvent = unknown;
          break;
        case requestSlaveReady:  //checked
          interrupt11Allowed = false;
          masterCommand = aux;
          nextEvent = unknown;
          break;
        case requestResetSlave:
          EEPROM.write(4094, 1);
          delay(6);
          resetFunc();
          break;
        case informSlaveToStart:
          interrupt11Allowed = true;
          masterCommand = aux;
          nextEvent = unknown;
          break;
        default:
          interrupt11Allowed = true;
          masterCommand = 0; //
          nextEvent = unknown;
          break;

      }
    }
  }
  else if (nextEvent == dashboardInfoData)
  {
    dashboardInfo = Wire.read();
    statusChanged = true;
    nextEvent = unknown;
  }

  digitalWrite(receiveDataLEDPin, LOW);
}

void requestEvent()                                //This Function is called when Master wants value from slave
{

  digitalWrite(requestDataLEDPin, HIGH);
  requestEventCalled = true;
  switch (masterCommand)
  {
    case requestSpreaderHeight:                                      // request spreader height
      byteSlaveAnswer = (findSpreaderHeight() & 0x00FF);
      Wire.write(byteSlaveAnswer);
      interrupt11Allowed = true;
      break;
    case requestSlaveReady:                                      // request ready state of slave, checked
      byteSlaveAnswer = slaveReadyStatus;
      Wire.write(byteSlaveAnswer);
      interrupt11Allowed = true;
      break;
    case requestInterruptType:                                      // request what type of interrupt was triggered
      byteSlaveAnswer = interruptType;
      Wire.write(byteSlaveAnswer);
      interruptType = noMessage;                  // in case of erratic interrupt on Master interrupt pin which results in a interrupt type request
      interrupt11Allowed = true;
      break;
    case requestDisplayLines:
      lineNumber++;
      if (lineNumber == 1)
      {
        Wire.write(lineOne, 16);
      }
      else
      {
        Wire.write(lineTwo, 16);
        lineNumber = 0;
        interrupt11Allowed = true;
      }
      break;
    default:
      byteSlaveAnswer = noMessage;                 // in case of an unidentified request from master
      Wire.write(byteSlaveAnswer);
      interrupt11Allowed = true;
      break;
  }
  digitalWrite(requestDataLEDPin, LOW);
}

void setDashboardLED (int Info, int Bit, byte LEDnr)
{
  if ((Info & Bit) != 0) {

    uint32_t One   = (NEOColors[LEDnr] & 0xFF0000) >> 16;
    uint32_t Two   = (NEOColors[LEDnr] & 0x00FF00) >> 8;
    uint32_t Three = (NEOColors[LEDnr] & 0x0000FF);
    strip.setPixelColor(LEDnr, One, Two, Three);
    strip.show(); //Laat de kleur zien.
  }
  else
  {
    strip.setPixelColor(LEDnr, 0, 0, 0);
    strip.show(); //blank LD in NEOPixel
  }

}

void setup() {
  bool switchLeds = false;
  int runNumber;

  strip.begin();
  strip.show();  //turn off all pixels.
  strip.setBrightness(50);
  dashboardInfo = 0;
  lightXmasTree();
  Serial.begin(9600);
  _SERIAL_PRINTLN("");
  _SERIAL_PRINTLN("=========================");
  _SERIAL_PRINTLN("Slave started");

  runNumber = EEPROM.read(4095) << 8 | EEPROM.read(4096);
  runNumber = runNumber + 1;
  EEPROM.write(4095, (runNumber >> 8));
  EEPROM.write(4096, (runNumber & 0xFF));

  pinMode(masterReadyForSlavePin, INPUT);
  heartStillBeating = (digitalRead(masterReadyForSlavePin) == HIGH);

  pinMode(spreaderTouchPin, INPUT);
  pinMode(receiveDataLEDPin, OUTPUT);
  pinMode(requestDataLEDPin, OUTPUT);
  Wire.begin(NODE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Initialise the LCD
  byteSlaveAnswer = slaveBusy;
  startTesttime = millis();

  digitalWrite(receiveDataLEDPin, LOW);
  digitalWrite(requestDataLEDPin, LOW);
  digitalWrite(receiveDataLEDPin, HIGH);
  delay(500);
  digitalWrite(receiveDataLEDPin, LOW);
  digitalWrite(requestDataLEDPin, HIGH);
  delay(500);
  digitalWrite(requestDataLEDPin, LOW);

  if (!heartStillBeating)
  {
    //    while ((masterCommand != 39 ))
    //    {
    _SERIAL_PRINT("Waiting for Master to initialize: ");
    _SERIAL_PRINTLN(millis());
    delay(1000);
  }
  _SERIAL_PRINTLN("Master ready - continue with slave");
  //  }

  pinMode(interruptSpreaderPin, OUTPUT);
  pinMode(SROSpreaderTriggerPin, OUTPUT);
  pinMode(SROSpreaderEchoPin, INPUT);

  pinMode(interruptPin, OUTPUT);
  pinMode(SROTriggerPin, OUTPUT);
  pinMode(SROEchoPin, INPUT);
  pinMode(interruptSpreaderMotorCountPin, INPUT);
  pinMode(spreaderTouchPin, INPUT);

  testNEOPixel();
  strcpy(lineOne, "Step 3 -slave   ");
  strcpy(lineTwo, "NEOPIXEL Test   ");
  _SERIAL_PRINTLN(lineOne);
  _SERIAL_PRINTLN(lineTwo);

  // prepare christmas tree

  statusChanged = true;
  dashboardInfo = 128;
  orientationstatusChanged = false;
  previousorientation = 0.0;
  digitalWrite(interruptPin, LOW);
  lightXmasTree();

  //
  // initialise collision prevention sensor
  //
  distance = 0;
  for (int i = 0; i < 9; i++)
  {
    distance = distance + findDistance();
    delay(50);
  }
  strcpy(lineOne, "Step 4 -slave   ");
  sprintf(lineTwo, "hor dist: %d", distance / 10);
  nearestObjectDistance = distance / 10;
  _SERIAL_PRINTLN(lineOne);
  _SERIAL_PRINTLN(lineTwo);

  spreaderHeight = 0;
  for (int i = 0; i < 5; i++)
  {
    spreaderHeight = spreaderHeight + findSpreaderHeight();
  }
  strcpy(lineOne, "Step 5-slave    ");
  sprintf(lineTwo, "spreader at %d", spreaderHeight / 5);
  _SERIAL_PRINT(lineOne);
  _SERIAL_PRINTLN(lineTwo);
  _SERIAL_PRINTLN("Handover control to Master after input from Serial port");
  interruptType = slaveReady;
  strcpy(lineOne, "Step 7-slave    ");
  strcpy(lineTwo, "Cnt to Master   ");
  delay(5000);
  triggerInterruptLocal(interruptPin);
  // go to sleep to give the master time to pick up the slave-ready command

}

void handleSpreaderAction()
{
  //
  // if spreader at the top, spreader can only go down; if spreader at bottom, spreader should only go up
  //
  bool spreaderIsLoaded = false;
  int spreaderHeightStart;
  int currentSpreaderHeight;
  spreaderHeightStart = findSpreaderHeight();

  Serial.print("currentSpreaderState = ");
  Serial.println(currentSpreaderState);
  Serial.print("requiredSpreaderPosition = ");
  Serial.println(requiredSpreaderPosition);

  if ((currentSpreaderState != atTop) && (requiredSpreaderPosition <= 70))
  {
    // going up
    _SERIAL_PRINTLN("Spreader to move up at least 1.5 cm");
    attachInterrupt(digitalPinToInterrupt(interruptSpreaderMotorCountPin), handleSpreaderMotorSensor, RISING);
    spreaderCount = 0;
    spreaderMoveDistance = spreaderHeightStart - 50;
    numberOfPulsesGoingUp = 0;
    while (abs(spreaderHeightStart - currentSpreaderHeight) < 15)
    {
      currentSpreaderHeight = findSpreaderHeight();
      _SERIAL_PRINT("Spreader at: ");
      _SERIAL_PRINT(currentSpreaderHeight);
      _SERIAL_PRINTLN(" mm");
      delay(20);
    }
    currentSpreaderHeight = findSpreaderHeight();
    //
    // check if spreader is loaded
    //
    spreaderIsLoaded = (digitalRead(spreaderTouchPin) == HIGH);

    //    auxCalculate = 20.0 * (4.0 * ((double)spreaderMoveDistance)) / mmPerRotation;
    //    requiredSpreaderCount = (int)auxCalculate;

    if (spreaderIsLoaded)
    {
      _SERIAL_PRINTLN("Spreader is loaded");
      while (currentSpreaderState != atTop)
      {
        currentSpreaderHeight = findSpreaderHeight();
        _SERIAL_PRINT("Spreader at: ");
        _SERIAL_PRINT(currentSpreaderHeight);
        _SERIAL_PRINTLN(" mm");
        delay(20);
      }
      currentSpreaderHeight = findSpreaderHeight();
      _SERIAL_PRINTLN("hit highest point");
    }
    else
    {
      //
      // spreader is not loaded, so go up until it hits the barrier
      _SERIAL_PRINTLN("Spreader is not loaded");
      spreaderSuspended = true;
      while (spreaderSuspended)
      {
        currentSpreaderHeight = findSpreaderHeight();
        _SERIAL_PRINT("Spreader at: ");
        _SERIAL_PRINT(currentSpreaderHeight);
        _SERIAL_PRINTLN(" mm");
        spreaderSuspended = (digitalRead(spreaderTouchPin) == LOW);
        delay(20);
      }
      _SERIAL_PRINTLN("hit switch");
    }
    numberOfPulsesGoingUp = spreaderCount;
    detachInterrupt(digitalPinToInterrupt(interruptSpreaderMotorCountPin));
  }
  else if ((currentSpreaderState != atBottom) && (requiredSpreaderPosition > 70))
  {
    // going down
    _SERIAL_PRINTLN("Spreader to move down at least 1.5 cm");
    attachInterrupt(digitalPinToInterrupt(interruptSpreaderMotorCountPin), handleSpreaderMotorSensor, RISING);
    spreaderCount = 0;
    spreaderMoveDistance = 220 - spreaderHeightStart;
    currentSpreaderHeight = spreaderHeightStart;
    while ((currentSpreaderHeight - spreaderHeightStart) < 15)
    {
      currentSpreaderHeight = findSpreaderHeight();
      _SERIAL_PRINT("Spreader at: ");
      _SERIAL_PRINT(currentSpreaderHeight);
      _SERIAL_PRINTLN(" mm");
      delay(20);
    }
    spreaderIsLoaded = (digitalRead(spreaderTouchPin) == HIGH);
    //
    // if spreader is loaded, just go down until numberOfpulses is gone
    //
    if (spreaderIsLoaded)
    {
      _SERIAL_PRINTLN("Spreader is loaded");
      while (spreaderCount < numberOfPulsesGoingUp)
      {
        currentSpreaderHeight = findSpreaderHeight();
        _SERIAL_PRINT("Spreader at: ");
        _SERIAL_PRINT(currentSpreaderHeight);
        _SERIAL_PRINT(" mm, Pulses: ");
        _SERIAL_PRINT(spreaderCount);
        _SERIAL_PRINT(" of ");
        _SERIAL_PRINTLN(numberOfPulsesGoingUp);
        delay(20);
      }
      _SERIAL_PRINTLN("Spreader gone down just as much as going up");
    }
    else
    {
      //
      // spreader is not loaded, so drop down until it hits a container or is deep enough
      _SERIAL_PRINTLN("Spreader is not loaded");
      spreaderSuspended = true;
      while (spreaderSuspended &&  (findSpreaderHeight() < 200))
      {
        spreaderSuspended = (digitalRead(spreaderTouchPin) == LOW);
        delay(50);
      }
      if (!spreaderSuspended)
      {
        _SERIAL_PRINTLN("Spreader hit switch");
      }
      else
      {
        _SERIAL_PRINTLN("Spreader at lowest point");
      }
    }

    detachInterrupt(digitalPinToInterrupt(interruptSpreaderMotorCountPin));
  }
  //
  // stop motor and set requiredSpreaderPosition to 0
  //
  triggerInterruptLocal(interruptSpreaderPin);
  requiredSpreaderPosition = 0;
}

void loop()
{
  //
  // status info received from master
  //
  if (receiveEventCalled)
  {
    receiveEventCalled = false;
    digitalWrite(receiveDataLEDPin, LOW);
  }
  if (requestEventCalled)
  {
    requestEventCalled = false;
    digitalWrite(requestDataLEDPin, LOW);
  }
  if (statusChanged)
  {
    lightXmasTree();
    statusChanged = false;
  }

  if (requiredSpreaderPosition != 0)
  {
    handleSpreaderAction();
  }

  nearestObjectDistance = (int)(0.9 * (double)nearestObjectDistance + 0.1 * (double)findDistance()); // high noise filter

  if ((nearestObjectDistance > 4) && (nearestObjectDistance < 20))
  {
    interruptType = obstacleInterrupt;
    triggerInterruptLocal(interruptPin);
  }
}
