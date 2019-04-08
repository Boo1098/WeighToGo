#include <Adafruit_MotorShield.h>
#include <Enes100.h>
#include <Wire.h>
#include <math.h>
#include "HX711.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Setup Constants
#define MIN_SPEED_TURN 60.0
#define OBSTACLE_TRIGGER_DISTANCE 30.0
#define DESTINATION_BUFFER_DISTANCE 0.1
#define MAX_SPEED 255.0
#define DRIVE_FAR_kP 255.0 * 5.0 / 3.14
#define ORIENT_kP (255.0 - MIN_SPEED_TURN) / 3.14

// Amount of time in ms that each loop waits
#define LOOP_WAIT 100

// Trigger pin of ultrasonic sensor
#define TRIG_PIN 12
// Echo pin of ultrasonic sensor
#define ECHO_PIN 11
// Load cell pins
#define DOUT 3
#define CLK 2

// I'm lazy
#define desX Enes100.destination.x
#define desY Enes100.destination.y
#define locX Enes100.location.x
#define locY Enes100.location.y
#define locT Enes100.location.theta
// #define Eprint Enes100.print
// #define Eprintln Enes100.println
#define updateLocation Enes100.updateLocation

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create the 4 motors on the OSV
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

HX711 scale;

float calibration_factor = -242;

void setup() {
  // // Team Name, Mission Type, Marker ID, RX Pin, TX Pin
  delay(5000);

  while (!Enes100.begin("Weigh to go", DEBRIS, 5, 7, 6)) {
    // Eprintln("Waiting for Connection.");
  }

  delay(500);

  Enes100.println("Connected!");

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare();  //Reset the scale to 0

  // long zero_factor = scale.read_average();

  // Eprint out destination location
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

  long start = millis();
  while (millis() - start < 2500) {
    updateEverything();
  }

  // Orient robot towards target.
  startUp();

  // // Drive to the target close enough.
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, locY, true);

  // // Point towards final target.
  orient(angleTo(desX, desY));

  // // Drive up close.
  float dist = distanceTo(desX, desY);
  if (dist > DESTINATION_BUFFER_DISTANCE) {
    driveFar(locX + (desX - locX) * (dist - DESTINATION_BUFFER_DISTANCE) / dist,
             locY + (desY - locY) * (dist - DESTINATION_BUFFER_DISTANCE) / dist, false);
  }
}

// Drives to a point on the field with obstacle avoidance if obsCheck = true.
void driveFar(double x, double y, bool obsCheck) {
  Enes100.print("Driving to ");
  Enes100.print(x);
  Enes100.print(", ");
  Enes100.println(y);
  bool flag = false;
  while (!flag) {
    updateEverything();
    if (obsCheck && obstacle()) {
      Enes100.println("Obstacle Found!");
      flag = true;
      avoidObstacle();
    } else {
      double leftSpeed = 255;
      double rightSpeed = 255;
      double theta = locT - angleTo(x, y);
      Enes100.print("error: ");
      // abs is evil.
      double atheta = theta;
      if (atheta < 0) {
        atheta *= -1.0;
      }
      Enes100.println(atheta);

      if (atheta > 0.01) {
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
        Enes100.println("Drived!");
        flag = true;
        leftSpeed = 0;
        rightSpeed = 0;
      }
      Enes100.print("left: ");
      Enes100.print(leftSpeed);
      Enes100.print(", right: ");
      Enes100.println(rightSpeed);
      setMotorSpeed(leftSpeed, rightSpeed);
    }
    // Delay for reasons?
    delay(LOOP_WAIT);
  }
}

// Run first to get robot aligned to a column.
void startUp() {
  Enes100.println("Initiating Launch Sequence.");
  updateEverything();

  // Depending on column, drives to center of nearest.
  if (locY < 0.66) {
    driveFar(locX + 0.1, 0.33, false);
  } else if (locY < 1.33) {
    driveFar(locX + 0.1, 1, false);
  } else {
    driveFar(locX + 0.1, 1.66, false);
  }

  // Orients self to get ready.
  orient(0);

  Enes100.println("All systems go.");
}

// Drives robot around an obstacle
void avoidObstacle() {
  Enes100.println("Avoiding Obstacle!");
  updateEverything();
  double newY = 0;
  if (locY > 1.333) {
    orient(-3.14 / 2.0);
    updateEverything();
    newY = 1;
    driveFar(locX, 1 + 0.333, true);
    driveFar(locX + .4, 1, true);
  } else if (locY > 1) {
    orient(3.14 / 2.0);
    updateEverything();
    newY = 1.666;
    driveFar(locX, 1.666 - 0.333, true);
    driveFar(locX + .4, 1.666, true);
  } else if (locY > 0.666) {
    orient(-3.14 / 2.0);
    updateEverything();
    newY = 0.333;
    driveFar(locX, 0.333 + 0.333, true);
    driveFar(locX + .4, 0.333, true);
  } else {
    orient(3.14 / 2.0);
    updateEverything();
    newY = 1;
    driveFar(locX, 1 - 0.333, true);
    driveFar(locX + .4, 1, true);
  }
  orient(0);
  Enes100.println("Obstacle Avoided!");
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
    Enes100.print("error: ");
    Enes100.println(error);
    // abs is evil.
    if (error < 0.01 && error > -0.01) {
      Enes100.println("Oriented!");
      flag = true;
      setMotorSpeed(0, 0);
    } else {
      double output = abs(error) * ORIENT_kP + MIN_SPEED_TURN;
      if (output > 255) {
        output = 255;
      }
      Enes100.print("output: ");
      Enes100.println(output);
      if (error > 0) {
        setMotorSpeed((int)output, (int)-output);
      } else {
        setMotorSpeed((int)-output, (int)output);
      }
    }
    delay(LOOP_WAIT);
  }
}

// unused
void loop() {}

// Returns ultrasonic distance
double getUltraDistance(int trig, int echo) {
  // magic
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return ((pulseIn(echo, HIGH) * .0343) / 2.0) * 10.0 / 11.0;
}

// Returns true if there is an obstacle detected.
bool obstacle() {
  return getUltraDistance(TRIG_PIN, ECHO_PIN) < OBSTACLE_TRIGGER_DISTANCE;
}

// Sets drive motors speeds
// -255<=speed<=255
void setMotorSpeed(int left, int right) {
  if (left < 0) {
    frontLeftMotor->run(BACKWARD);
    backLeftMotor->run(BACKWARD);
  } else {
    frontLeftMotor->run(FORWARD);
    backLeftMotor->run(FORWARD);
  }
  if (right < 0) {
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

double getWeight() {
  return scale.get_units();
}

void printStats() {
  Enes100.print("Location: ");
  Enes100.print(locX);
  Enes100.print(", ");
  Enes100.print(locY);
  Enes100.print(", ");
  Enes100.println(locT);
  Enes100.print("Sensors: ");
  // Enes100.print(getWeight());
  // Enes100.print("    ");
  Enes100.println(getUltraDistance(TRIG_PIN, ECHO_PIN));
}
