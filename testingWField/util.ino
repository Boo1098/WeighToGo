#include <Enes100.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Gives the angle relative to the horizontal from OSV to target.
float angleTo(float x, float y) {
    float delX = x-getX();
    float delY = y-getY();
    float angle= atan2(delY, delX);
    return angle;
}

// This function computes the distance from the OSV to the coordinate passed in
float distanceTo(float x, float y) {
    float delX = getX()-x;
    float delY = getY()-y;
    return sqrt(sq(delX)+sq(delY));
}

// Returns OSV X
float getX(){
  return locX;
}

// Returns OSV Y
float getY(){
  return locY;
}

// Returns OSV Theta
float getTheta(){
  return locT;
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

// Returns true if there is an obstacle detected.
bool obstacle(){
  return getUltraDistance(TRIG_PIN, ECHO_PIN)<OBSTACLE_TRIGGER_DISTANCE;
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