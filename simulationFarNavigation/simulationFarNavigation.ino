#include <math.h>
#include "Enes100Simulation.h"
#include "TankSimulation.h"

#define MIN_SPEED_TURN 50.0
#define OBSTACLE_TRIGGER_DISTANCE 0.1
#define DESTINATION_BUFFER_DISTANCE 0.3

// I'm lazy
#define Enes100 Enes100Simulation
#define desX Enes100.destination.x
#define desY Enes100.destination.y
#define locX Enes100.location.x
#define locY Enes100.location.y
#define locT Enes100.location.theta
#define print Enes100.print
#define println Enes100.println
#define updateLocation Enes100.updateLocation

void setup() {
  TankSimulation.begin();
  while (!Enes100.begin())
    ;

  println("Starting Navigation");

  while (!updateLocation()) {
    println("Unable to update Location");
  }

  printStats();

  // Orient robot towards target.
  avoidObstacle();

  // Drive to the target close enough.
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, getY());

  // Point towards final target.
  orient(angleTo(desX, desY));

  // Drive up close.
  float dist = distanceTo(desX, desY);
  driveFar(getX() + (desX - getX()) * DESTINATION_BUFFER_DISTANCE / dist,
           getY() + (desY - getY()) * DESTINATION_BUFFER_DISTANCE / dist);
}

void loop() {}

// Drives to a point on the field with obstacle avoidance.
void driveFar(float x, float y) {
  print("Driving to ");
  print(x);
  print(", ");
  println(y);
  bool flag = false;
  float kP = 255.0 * 10.0 / 3.14;
  while (!flag) {
    updateEverything();
    if (obstacle()) {
      println("Obstacle Found!");
      flag = true;
      avoidObstacle();
    } else {
      float leftSpeed = 255;
      float rightSpeed = 255;
      float theta = (getTheta() - angleTo(x, y));
      if (abs(theta) > 0.01) {
        if (theta > 0) {
          rightSpeed -= abs(kP * theta);
          if (rightSpeed < -255) {
            rightSpeed = -255;
          }
        } else {
          leftSpeed -= abs(kP * theta);
          if (leftSpeed < 0) {
            leftSpeed = -255;
          }
        }
      }
      if (distanceTo(x, y) < .05) {
        flag = true;
        leftSpeed = 0;
        rightSpeed = 0;
      }
      setSpeed(leftSpeed, rightSpeed);
    }
  }
}

// Returns true if there is an obstacle detected.
bool obstacle() { return getUltraDistance() < OBSTACLE_TRIGGER_DISTANCE; }

// Drives robot around an obstacle
void avoidObstacle() {
  println("Avoiding Obstacle!");
  bool flag = false;
  float kP = 255.0 * 10.0 / 3.14;
  updateEverything();
  float newY = 0;
  if (getY() > 1.333) {
    orient(-3.14 / 2.0);
    updateEverything();
    newY = 1;
    driveFar(getX(), 1 + 0.333);
    driveFar(getX() + .4, 1);
  } else if (getY() > 1) {
    orient(3.14 / 2.0);
    updateEverything();
    newY = 1.666;
    driveFar(getX(), 1.666 - 0.333);
    driveFar(getX() + .4, 1.666);
  } else if (getY() > 0.666) {
    orient(-3.14 / 2.0);
    updateEverything();
    newY = 0.333;
    driveFar(getX(), 0.333 + 0.333);
    driveFar(getX() + .4, 0.333);
  } else {
    orient(3.14 / 2.0);
    updateEverything();
    newY = 1;
    driveFar(getX(), 1 - 0.333);
    driveFar(getX() + .4, 1);
  }
  orient(0);
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, newY);
}

// Points robot towards specified angle.
// float t - An angle in radians
void orient(float t) {
  print("Orienting to ");
  println(t);
  bool flag = false;
  float kP = (255.0 - MIN_SPEED_TURN) / 3.14;
  while (!flag) {
    updateEverything();
    float theta = getTheta();
    if (abs(theta - t) < .01) {
      flag = true;
      setSpeed(0, 0);
    } else {
      float error = theta - t;
      float output = abs(error) * kP + MIN_SPEED_TURN;
      if (output > 255) {
        output = 255;
      }
      if (error > 0) {
        setSpeed(output, -output);
      } else {
        setSpeed(-output, output);
      }
    }
  }
}

void setSpeed(int left, int right) {
  TankSimulation.setLeftMotorPWM(left);
  TankSimulation.setRightMotorPWM(right);
}

float getUltraDistance() { return Enes100Simulation.readDistanceSensor(1); }

void printStats() {
  print("Location: ");
  print(getX());
  print(", ");
  print(getY());
  print(", ");
  println(getTheta());
  print("Destination: ");
  print(desX);
  print(", ");
  println(desY);
}
