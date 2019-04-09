#include <math.h>
#include "Enes100Simulation.h"
#include "TankSimulation.h"

#define MIN_SPEED_TURN 50.0
#define OBSTACLE_TRIGGER_DISTANCE 0.125
#define DESTINATION_BUFFER_DISTANCE 0.3
#define DRIVE_FAR_kP 255.0 * 15.0 / 3.14
#define ORIENT_kP (255.0 - MIN_SPEED_TURN) / 3.14

// I'm lazy
#define Enes100 Enes100Simulation
#define desX Enes100.destination.x
#define desY Enes100.destination.y
#define locX Enes100.location.x
#define locY Enes100.location.y
#define locT Enes100.location.theta
// #define print Enes100.print
// #define println Enes100.println
#define updateLocation Enes100.updateLocation

void setup() {
  TankSimulation.begin();
  while (!Enes100.begin())
    ;

  Enes100.println("Starting Navigation");

  while (!updateLocation()) {
    Enes100.println("Unable to update Location");
  }

  printStats();

  // Orient robot towards target.
  avoidObstacle();

  // Drive to the target close enough.
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, getY(), true);

  // Point towards final target.
  orient(angleTo(desX, desY));

  // Drive up close.
  float dist = distanceTo(desX, desY);
  driveFar(getX() + (desX - getX()) * DESTINATION_BUFFER_DISTANCE / dist,
           getY() + (desY - getY()) * DESTINATION_BUFFER_DISTANCE / dist, false);
}

void loop() {}

// Drives to a point on the field with obstacle avoidance if obsCheck = true.
void driveFar(double x, double y, bool obsCheck) {
  Enes100.print("Driving to ");
  Enes100.print(x);
  Enes100.print(", ");
  Enes100.println(y);
  bool flag = false;
  while (!flag) {
    updateEverything();
    if (obstacle() && obsCheck) {
      Enes100.println("Obstacle Found!");
      flag = true;
      avoidObstacle();
    } else {
      double leftSpeed = 255;
      double rightSpeed = 255;
      double theta = locT - angleTo(x, y);
      // Enes100.print("error: ");
      // Enes100.println(theta);
      if (abs(theta) > 0.01) {
        if (theta > 0) {
          rightSpeed -= abs(DRIVE_FAR_kP * theta);
          if (rightSpeed < -255) {
            rightSpeed = -255;
          }
        } else {
          leftSpeed -= abs(DRIVE_FAR_kP * theta);
          if (leftSpeed < -255) {
            leftSpeed = -255;
          }
        }
      }
      if (distanceTo(x, y) < .05) {
        flag = true;
        leftSpeed = 0;
        rightSpeed = 0;
      }
      // Enes100.print("left: ");
      // Enes100.print(leftSpeed);
      // Enes100.print(", right: ");
      // Enes100.println(rightSpeed);
      setMotorSpeed(leftSpeed, rightSpeed);
    }
  }
}

// Drives robot around an obstacle
void avoidObstacle() {
  Enes100.println("Avoiding Obstacle!");
  double newY = locY > 1.333 || locY < 0.666 ? 1 : locY > 1 ? 1.666 : 0.333;
  double middleY = locY > 1 ? 1.333 : 0.666;
  orient(locY > 1.333 || (locY < 1 && locY > 0.666) ? -1.57 : 1.57);
  driveFar(locX, middleY, false);
  driveFar(locX + 0.4, newY, false);
  // Re-orient forwards.
  orient(0);
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, newY, true);
}

// Points robot towards specified angle.
// double t - An angle in radians
void orient(double t) {
  Enes100.print("Orienting to ");
  Enes100.println(t);
  bool flag = false;
  while (!flag) {
    updateEverything();
    double theta = locT;
    double error = theta - t;
    // Enes100.print("error: ");
    // Enes100.println(error);
    if (abs(error) < .01) {
      Enes100.println("Oriented");
      flag = true;
      setMotorSpeed(0, 0);
    } else {
      double output = abs(error) * ORIENT_kP + MIN_SPEED_TURN;
      if (output > 255) {
        output = 255;
      }
      // Enes100.print("output: ");
      // Enes100.println(output);
      if (error > 0) {
        setMotorSpeed((int)output, (int)-output);
      } else {
        setMotorSpeed((int)-output, (int)output);
      }
    }
  }
}

// Returns true if there is an obstacle detected.
bool obstacle() { return getUltraDistance() < OBSTACLE_TRIGGER_DISTANCE; }

void setMotorSpeed(int left, int right) {
  TankSimulation.setLeftMotorPWM(left);
  TankSimulation.setRightMotorPWM(right);
}

float getUltraDistance() { return Enes100Simulation.readDistanceSensor(1); }

void printStats() {
  Enes100.print("Location: ");
  Enes100.print(getX());
  Enes100.print(", ");
  Enes100.print(getY());
  Enes100.print(", ");
  Enes100.println(getTheta());
  Enes100.print("Destination: ");
  Enes100.print(desX);
  Enes100.print(", ");
  Enes100.println(desY);
}
