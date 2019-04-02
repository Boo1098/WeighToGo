#include "Enes100Simulation.h"
#include "TankSimulation.h"
#include <math.h>

#define MIN_SPEED_TURN 50.0
#define OBSTACLE_TRIGGER_DISTANCE 0.1
#define DESTINATION_BUFFER_DISTANCE 0.3

// I'm lazy
#define desX Enes100Simulation.destination.x
#define desY Enes100Simulation.destination.y
#define locX Enes100Simulation.location.x
#define locY Enes100Simulation.location.y
#define locT Enes100Simulation.location.theta

void setup() {
  TankSimulation.begin();
  while(!Enes100Simulation.begin());

  Enes100Simulation.println("Starting Navigation");

  while (!Enes100Simulation.updateLocation()) {
    Enes100Simulation.println("Unable to update Location");
  }
  
  printStats();

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

void loop() {
  
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

// Returns true if there is an obstacle detected.
bool obstacle(){
  return getUltraDistance()<OBSTACLE_TRIGGER_DISTANCE;
}

// Drives robot around an obstacle
void avoidObstacle(){
  Enes100Simulation.println("Avoiding Obstacle!");
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
  Enes100Simulation.print("Orienting to ");
  Enes100Simulation.println(t);
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

void setSpeed(int left, int right) {
  TankSimulation.setLeftMotorPWM(left);
  TankSimulation.setRightMotorPWM(right);
}

float getUltraDistance(){
  return Enes100Simulation.readDistanceSensor(1);
}

void updateEverything(){
  Enes100Simulation.updateLocation();
}

void printStats(){
  Enes100Simulation.print("Location: ");
  Enes100Simulation.print(getX());
  Enes100Simulation.print(", ");
  Enes100Simulation.println(getY());
  Enes100Simulation.print("Destination: ");
  Enes100Simulation.print(desX);
  Enes100Simulation.print(", ");
  Enes100Simulation.println(desY);
}

