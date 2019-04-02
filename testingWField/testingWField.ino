#include <Enes100.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Setup Constants
#define MIN_SPEED_TURN 50.0
#define OBSTACLE_TRIGGER_DISTANCE 0.1
#define DESTINATION_BUFFER_DISTANCE 0.3

// Trigger pin of ultrasonic sensor
#define TRIG_PIN 8 
// Echo pin of ultrasonic sensor
#define ECHO_PIN 7

// I'm lazy
#define desX Enes100.destination.x
#define desY Enes100.destination.y
#define locX Enes100.location.x
#define locY Enes100.location.y
#define locT Enes100.location.theta

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  

// Create the 4 motors on the OSV
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

void setup() {
  // Team Name, Mission Type, Marker ID, RX Pin, TX Pin
  Enes100.begin("Weigh to go", DEBRIS, 3, 8, 9);

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
  
  // Start Navigation Code

  //Orient robot towards target.
  orient(0);

  //Drive to the target close enough.
  driveFar(desX-DESTINATION_BUFFER_DISTANCE, getY());

  //Point towards final target.
  orient(angleTo(desX, desY));

  //Drive up close.
  float dist = distanceTo(desX, desY);
  driveFar(getX()+(desX-getX())*0.25/dist,getY()+(desY-getY())*0.25/dist);
}

// Drives to a point on the field with obstacle avoidance.
void driveFar(float x, float y) {
  Enes100.print("Driving to ");
  Enes100.print(x);
  Enes100.print(", ");
  Enes100.println(y);
  bool flag = false;
  float kP = 255.0*10.0/3.14;
  while(!flag){
    updateEverything();
    if(obstacle()){
      Enes100.println("Obstacle Found!");
      flag = true;
      avoidObstacle();
    } else {
      float leftSpeed = 255;
      float rightSpeed = 255;
      float theta = (getTheta() - angleTo(x, y));
      if(abs(theta)>0.01){
        if(theta>0){
          rightSpeed-=abs(kP*theta);
          if(rightSpeed<-255){
            rightSpeed=-255;
          }
        } else {
          leftSpeed-=abs(kP*theta);
          if(leftSpeed<0){
            leftSpeed=-255;
          }
        }
      }
      if(distanceTo(x,y)<.05){
        flag = true;
        leftSpeed = 0;
        rightSpeed=0;
      }
      setSpeed(leftSpeed,rightSpeed);
    }
  }
}

// Drives robot around an obstacle
void avoidObstacle(){
  Enes100.println("Avoiding Obstacle!");
  bool flag = false;
  float kP = 255.0*10.0/3.14;
  updateEverything();
  float newY = 0;
  if(getY()>1.333 ){
    orient(-3.14/2.0);
    updateEverything();
    newY=1;
    driveFar(getX(), 1+0.333);
    driveFar(getX()+.4, 1);
  } else if (getY()>1) {
    orient(3.14/2.0);
    updateEverything();
    newY=1.666;
    driveFar(getX(), 1.666-0.333);
    driveFar(getX()+.4, 1.666);
  } else if (getY()>0.666){
    orient(-3.14/2.0);
    updateEverything();
    newY=0.333;
    driveFar(getX(), 0.333+0.333);
    driveFar(getX()+.4, 0.333);
  } else {
    orient(3.14/2.0);
    updateEverything();
    newY=1;
    driveFar(getX(), 1-0.333);
    driveFar(getX()+.4, 1);
  }
  orient(0);
  driveFar(desX-DESTINATION_BUFFER_DISTANCE,newY);
}

// Points robot towards specified angle.
// float t - An angle in radians
void orient(float t) {
  Enes100.print("Orienting to ");
  Enes100.println(t);
  bool flag = false;
  float kP = (255.0-MIN_SPEED_TURN)/3.14;
  while(!flag){
    updateEverything();
    float theta = getTheta();
    if(abs(theta-t)<.01){
      flag = true;
      setSpeed(0,0);
    } else {
      float error = theta-t;
      float output = abs(error)*kP+MIN_SPEED_TURN;
      if(output>255){
        output = 255;
      }
      if(error>0){
        setSpeed(output,-output);
      } else{
        setSpeed(-output,output);
      }
    }
  }
}

// Anything that needs to be ran every tick goes here
void updateEverything(){
  Enes100.updateLocation();
}

//unused
void loop() {
  
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