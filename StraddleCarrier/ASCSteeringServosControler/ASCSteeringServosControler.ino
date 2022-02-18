#include <Wire.h> // Using I2C communication to communicate between Arduino's on board
#include <Servo.h>
#define steeringMotor6Pin              6
#define steeringMotor1Pin              3
#define steeringMotor3Pin             5
#define steeringMotor4Pin             9

Servo motor1Servo;
Servo motor4Servo;
Servo motor3Servo;
Servo motor6Servo;
long angleRead;

long waiting()
{
  long aux;
  while (!Serial.available());
  aux = Serial.parseInt();

  return aux; 
  
}

void setup() {
  char aux;
  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(steeringMotor1Pin, OUTPUT);
  pinMode(steeringMotor4Pin, OUTPUT);
  pinMode(steeringMotor3Pin, OUTPUT);
  pinMode(steeringMotor6Pin, OUTPUT);
  motor1Servo.attach(steeringMotor1Pin);
  motor4Servo.attach(steeringMotor4Pin);
  motor3Servo.attach(steeringMotor3Pin);
  motor6Servo.attach(steeringMotor6Pin);
  Serial.println("Press enter to start");
  angleRead = waiting();
}

void loop() {

  Serial.println("AngleMotors?");
  angleRead = waiting();
  motor1Servo.write(angleRead);
  motor3Servo.write(angleRead);
  motor4Servo.write(angleRead);
  motor6Servo.write(angleRead);
}
