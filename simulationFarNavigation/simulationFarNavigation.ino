#include "Enes100Simulation.h"
#include "TankSimulation.h"
#include <math.h>

#define MIN_SPEED_TURN 50.0
#define DESTINATION_BUFFER_DISTANCE 0.3

void setup() {
  TankSimulation.begin();
  while(Enes100Simulation.begin());

  Enes100Simulation.println("Starting Navigation");

  while (!Enes100Simulation.updateLocation()) {
    Enes100Simulation.println("Unable to update Location");
  }
  
  printStats();

  //Orient robot towards target.
  orient(0);

  //Drive to the target close enough.
  driveFar(Enes100Simulation.destination.x-DESTINATION_BUFFER_DISTANCE, Enes100Simulation.location.y);

  //Point towards final target.
  orient(angleTo(Enes100Simulation.destination.x, Enes100Simulation.destination.y));

  //Drive up close.
  float dist = distanceTo(Enes100Simulation.destination.x, Enes100Simulation.destination.y);
  driveFar(Enes100Simulation.location.x+(Enes100Simulation.destination.x-Enes100Simulation.location.x)*0.25/dist,Enes100Simulation.location.y+(Enes100Simulation.destination.y-Enes100Simulation.location.y)*0.25/dist);
}

void loop() {
  
}

// Gives the angle relative to the horizontal from OSV to target.
float angleTo(float x, float y) {
    float delX = x-Enes100Simulation.location.x;
    float delY = y-Enes100Simulation.location.y;
    float angle= atan2(delY, delX);
    return angle;
}

// This function computes the distance from the OSV to the coordinate passed in
float distanceTo(float x, float y) {
    float delX = Enes100Simulation.location.x-x;
    float delY = Enes100Simulation.location.y-y;
    return sqrt(sq(delX)+sq(delY));
}

// Drives to a point on the field with obstacle avoidance.
void driveFar(float x, float y) {
  Enes100Simulation.print("Driving to ");
  Enes100Simulation.print(x);
  Enes100Simulation.print(", ");
  Enes100Simulation.println(y);
  bool flag = false;
  float kP = 255.0*10.0/3.14;
  while(!flag){
    updateEverything();
    if(obstacle()){
      Enes100Simulation.println("Obstacle Found!");
      flag = true;
      avoidObstacle();
    } else {
      float leftSpeed = 255;
      float rightSpeed = 255;
      float theta = (Enes100Simulation.location.theta - angleTo(x, y));
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
      setMotors(leftSpeed,rightSpeed);
    }
  }
}

// Returns true if there is an obstacle detected.
bool obstacle(){
  return (getLeftDistance()<0.1) || (getRightDistance()<0.1);
}

// Drives robot around an obstacle
void avoidObstacle(){
  Enes100Simulation.println("Avoiding Obstacle!");
  bool flag = false;
  float kP = 255.0*10.0/3.14;
  updateEverything();
  float newY = 0;
  if(Enes100Simulation.location.y>1.333 ){
    orient(-3.14/2.0);
    updateEverything();
    newY=1;
    driveFar(Enes100Simulation.location.x, 1+0.333);
    driveFar(Enes100Simulation.location.x+.4, 1);
  } else if (Enes100Simulation.location.y>1) {
    orient(3.14/2.0);
    updateEverything();
    newY=1.666;
    driveFar(Enes100Simulation.location.x, 1.666-0.333);
    driveFar(Enes100Simulation.location.x+.4, 1.666);
  } else if (Enes100Simulation.location.y>0.666){
    orient(-3.14/2.0);
    updateEverything();
    newY=0.333;
    driveFar(Enes100Simulation.location.x, 0.333+0.333);
    driveFar(Enes100Simulation.location.x+.4, 0.333);
  } else {
    orient(3.14/2.0);
    updateEverything();
    newY=1;
    driveFar(Enes100Simulation.location.x, 1-0.333);
    driveFar(Enes100Simulation.location.x+.4, 1);
  }
  orient(0);
  driveFar(Enes100Simulation.destination.x-DESTINATION_BUFFER_DISTANCE,newY);
}

// Points robot towards specified angle.
// float t - An angle in radians
void orient(float t) {
  Enes100Simulation.print("Orienting to ");
  Enes100Simulation.println(t);
  bool flag = false;
  float kP = (255.0-MIN_SPEED_TURN)/3.14;
  while(!flag){
    updateEverything();
    float theta = Enes100Simulation.location.theta;
    if(abs(theta-t)<.01){
      flag = true;
      setMotors(0,0);
    } else {
      float error = theta-t;
      float output = abs(error)*kP+MIN_SPEED_TURN;
      if(output>255){
        output = 255;
      }
      if(error>0){
        setMotors(output,-output);
      } else{
        setMotors(-output,output);
      }
    }
  }
}

void setMotors(int left, int right) {
  TankSimulation.setLeftMotorPWM(left);
  TankSimulation.setRightMotorPWM(right);
}

float getLeftDistance(){
  return Enes100Simulation.readDistanceSensor(0);
}

float getRightDistance(){
  return Enes100Simulation.readDistanceSensor(2);
}

void updateEverything(){
  Enes100Simulation.updateLocation();
}

void printStats(){
  Enes100Simulation.print("Location: ");
  Enes100Simulation.print(Enes100Simulation.location.x);
  Enes100Simulation.print(", ");
  Enes100Simulation.println(Enes100Simulation.location.y);
  Enes100Simulation.print("Destination: ");
  Enes100Simulation.print(Enes100Simulation.destination.x);
  Enes100Simulation.print(", ");
  Enes100Simulation.println(Enes100Simulation.destination.y);
}

