#include <Enes100.h>
#include <TankSimulation.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Trigger pin of ultrasonic sensor
#define TRIG_PIN 8 
// Echo pin of ultrasonic sensor
#define ECHO_PIN 7

// I'm lazy
#define desX Enes100.destination.x
#define desY Enes100.destination.y

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  

// Create the 4 motors on the OSV
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

void setup() {
  // Team Name, Mission Type, Marker ID, RX Pin, TX Pin
  Enes100.begin("Weight to go", DEBRIS, 3, 8, 9);

  // Print out destination location
  Enes100.print("Destination is at (");
  Enes100.print(desX);
  Enes100.print(", ");
  Enes100.print(desY);
  Enes100.println(")");

  // Set pin modes of ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);

  // Start Motor Controller.
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  while(true){
    updateEverything();
  }
}

// Anything that needs to be ran every tick goes here
void updateEverything(){
  Enes100.updateLocation();
}

// Returns distance from OSV to (x,y) location
float getDistance(float x, float y){
  return sqrt(sq(x-getX())+sq(y-getY()));
}

// Returns angle between Horizontal, OSV location, and target (x,y)
float getAngle(float x, float y){
  float delX = x-getX();
  float delY = y-getY();
  return atan2(delY,delX);
}

// Returns OSV X
float getX(){
  return Enes100.location.x;
}

// Returns OSV Y
float getY(){
  return Enes100.location.y;
}

// Returns OSV Theta
float getTheta(){
  return Enes100.location.theta;
}

// Returns ultrasonic distance
float getUltraDistance(int trig, int echo){
  digitalWrite(trig, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);
  return ((pulseIn(echo, HIGH)*.0343)/2.0)*10.0/11.0;
}

// Sets drive motors speeds
// -255<=speed<=255
void setSpeed(int left, int right){
  if(left<0){
    frontLeftMotor->run(BACKWARD);
    backLeftMotor->run(BACKWARD);
  } else {
    frontLeftMotor->run(FORWARD);
    backLeftMotor->run(FORWARD);
  }
  if(right<0){
    frontRightMotor->run(BACKWARD);
    backRightMotor->run(BACKWARD);
  } else {
    frontRightMotor->run(FORWARD);
    backRightMotor->run(FORWARD);
  }
  
  backRightMotor->setSpeed(abs(right));
  frontRightMotor->setSpeed(abs(right));
  frontLeftMotor->setSpeed(abs(left));
  backLeftMotor->setSpeed(abs(left));
}

//unused
void loop() {
  
}
