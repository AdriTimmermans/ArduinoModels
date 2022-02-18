// connect motor controller pins to Arduino digital pins
// functions:
// 1. read IR receiver front of vehicle
// 2. generate interrupt on master
// 3. reply on request for IRcommand data via I2C
// 4. read left and right motor encoder disc and calculate frequency
// 5. reply on request for motor frequencies via I2C
//
//======================= Declarations ===================================================
//
int pulseRightPin   = 2;      // interrupt input pin from right engine monitor
int pulseLeftPin    = 3;      // interrupt input pin from left engine monitor
int receiverPin     = 7;      // function 1
int interruptPin    = 9;      // generates an interrupt on pin 2 of master

#include <IRremote.h> 
// IR initialisation

IRrecv irrecv(receiverPin); // create instance of 'irrecv'
decode_results results;

volatile long counterLeft        = 0;      // pulse count from leftmotor since previous start
volatile long counterRight       = 0;      // pulse count from leftmotor since previous start
volatile int leftMotorFrequency  = 0;      // calculated frequency left motor
volatile int rightMotorFrequency = 0;      // calculated frequency right motor

#include <Wire.h>                          // I2C
#include <TimerOne.h>
volatile long millisecondsNow;                      // used for calculating frequency
volatile long millisecondsThen;                     // used for calculating frequency

// operational parameters:
int requestType;                           // used to determine what kind of data the master wants
//
//======================= Modules ========================================================
//
void monitorLeftMotor() // counts from LM393IR from wheels
{
  // interupt function to count pulses of left motor
  counterLeft++;
}

void monitorRightMotor() // counts from LM393IR from wheels
{
  // interupt function to count pulses of right motor
  counterRight++;
}

void irReceive()
{
  // module to capture the Remote Control button that is pressed and signal to the master we have data ready
   if (irrecv.decode(&results))
   {
     Serial.print("Generate interrupt for code:");
     Serial.println(results.value, HEX);
     digitalWrite (interruptPin, HIGH); // 
     delay (10); // delay 10ms
     digitalWrite (interruptPin, LOW); //
     irrecv.resume() ;
  }
}

void announceRequestType()
{
  // this module is called before a specific request type is send to announce what kind of data the master will ask for next
  // other data is ignored
  byte high;
  byte low;

  Serial.print("Signal from Master received");
  Serial.print("Number of bytes: ");
  Serial.print(Wire.available());
  if (Wire.available() == 2)
  {
    high = Wire.read();
    low = Wire.read();
    Serial.print("high byte:");
    Serial.print(high);
    Serial.print("\t low byte:");
    Serial.print(low);
    Serial.print("\t resulting in ");
    requestType = high << 8 | low;
    Serial.print("Integer on I2C:");
    Serial.println(requestType);
  }
  else
  {
   flushWire();
  }
}

void flushWire()
{
// unexpected data on I2C, skip it
  byte flushByte;

  while (Wire.available()) 
  {
    flushByte = Wire.read();
  }
}

void timerFrequency()
{
// this function is called after the timer has run out (now 2 seconds)
// the function will calculate the frequency of each motor and keep that until the Master send a request for it
  float aux;
  Timer1.detachInterrupt(); // stop the timer
  millisecondsThen = millisecondsNow;
  millisecondsNow = millis();
  aux = 1000.0 / (float)(millisecondsNow - millisecondsThen);
  leftMotorFrequency =  (int)(counterLeft * aux) / 20; 
  rightMotorFrequency = (int)(counterRight * aux) / 20;
//  Serial.print("Left counter: ");
//  Serial.print(counterLeft);
//  Serial.print("\ frequency: ");
//  Serial.print(leftMotorFrequency);
//  Serial.print("\t Right counter: ");
//  Serial.print(counterRight);
//  Serial.print("\t frequency: ");
//  Serial.println(rightMotorFrequency);
  counterLeft = 0;
  counterRight = 0;
  Timer1.attachInterrupt(timerFrequency); // enable timer again
}

void requestEvent()
{
  byte b[5];
  // master sends a request for data, the previous call determined the kind of data that is required.
  Serial.print("Data request received from master using request type: ");
  Serial.println (requestType);
  
  switch (requestType)
  {
    case 100:
      b[1] = (results.value & 0xFF000000) >> 24;
      b[2] = (results.value & 0x00FF0000) >> 16;
      b[3] = (results.value & 0x0000FF00) >> 8;
      b[4] = (results.value & 0x000000FF);
      Serial.print("Sending new command: ");
      Serial.print(results.value, HEX);
      Serial.print("-");
      Serial.print((results.value & 0xFF000000), HEX);
      Serial.print(",");
      Serial.print((results.value & 0x00FF0000), HEX);
      Serial.print(",");
      Serial.print((results.value & 0x0000FF00), HEX);
      Serial.print(",");
      Serial.print((results.value & 0x000000FF), HEX);
     
      Serial.print(" in bytes: ");
      for (int i=0;i<4;i++)
      {
        Serial.print(b[i+1], HEX);
        Serial.print(" - ");
        Wire.write(b[i+1]);
      }
      Serial.println();
      break;
    case 101:
      Wire.write(0);    
      Wire.write(leftMotorFrequency);
      Wire.write(0);
      Wire.write(rightMotorFrequency);
      Serial.print("Sending left freq: ");
      Serial.print(leftMotorFrequency);
      Serial.print(" right freq: ");
      Serial.println(rightMotorFrequency);
      break;
  }
}
//
//======================= Setup module - once =================================================
//
void setup()
{
  // initialisation is selfexplanatory
 
  Serial.begin(9600);
  Serial.println("Nano started");
  pinMode(interruptPin, OUTPUT);
  digitalWrite(interruptPin, LOW);   

  attachInterrupt(digitalPinToInterrupt(pulseLeftPin), monitorLeftMotor, RISING);   // attach pulse counter left motor
  attachInterrupt(digitalPinToInterrupt(pulseRightPin), monitorRightMotor, RISING); // attach pulse counter right motor
  
  Timer1.initialize(2000000);            // 2.000.000 micro seconds = 2 seconds
  Timer1.attachInterrupt(timerFrequency);

  Wire.begin(5);
  Wire.onReceive(announceRequestType);   // define function to be called when a byte is presented on the I2C bus
  Wire.onRequest(requestEvent);          // define function to be called when the Master requests for information (before this is called at the master, a message is sent describing the type of data requested
  millisecondsNow = millis();            // used to determine if the IR code coming in is a copy of the one previously received
  
  pinMode(receiverPin, INPUT);
  irrecv.enableIRIn();                   // start reading the tl1838 IR sensor
}
//
//======================= Loop module - repeat infinitely =====================================
//
void loop()
{
  // the main loop is monitoring the IR receiver. all other functions are governed by requests and data from I2C or the timer 
  irReceive();
}
