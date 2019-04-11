#include <Adafruit_MotorShield.h>
#include <Enes100.h>
#include <Wire.h>
#include <math.h>
#include "HX711.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

/*
* Setup Constants
*/
// Minimum speed required to turn
#define MIN_SPEED_TURN 60.0
// Distance in centimeters that triggers obstacle
#define OBSTACLE_TRIGGER_DISTANCE 30.0
// Distance we want the far navigation to end up from the mission site
#define DESTINATION_BUFFER_DISTANCE 0.1
// Max speed of OSV. Not currently implemented
#define MAX_SPEED 255.0
// Minimum speed OSV requires to move forwards
#define MIN_SPEED 50.0
// Proportional factor for driveFar corrections
#define DRIVE_FAR_kP 255.0 * 5.0 / 3.14
// Proportional factor for orient corrections
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
#define updateLocation Enes100.updateLocation

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create the 4 motors on the OSV
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

// Create scale object
HX711 scale;

// Magic
float calibration_factor = -242;

void setup() {
  delay(5000);

  // Wait for connection to vision system.
  // Team Name, Mission Type, Marker ID, RX Pin, TX Pin
  while (!Enes100.begin("Weigh to go", DEBRIS, 5, 6, 7)) {
    // Eprintln("Waiting for Connection.");
  }

  // shrug?
  delay(500);

  Enes100.println("Connected!");

  // Scale initilizaiton.
  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare();  //Reset the scale to 0

  // print out destination location
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

  // Prints location data for 2.5 seconds
  long start = millis();
  while (millis() - start < 2500) {
    updateEverything();
  }

  // Moves OSV to one of the three colunms.
  startUp();

  // Drive to the target close enough.
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, locY, true);

  // Point towards final target.
  orient(angleTo(desX, desY));

  // Drive up close.
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

  // Run a loop until interruption
  bool flag = false;
  while (!flag) {
    updateEverything();

    // Check for obstacle.
    if (obsCheck && obstacle()) {
      Enes100.println("Obstacle Found!");
      flag = true;
      avoidObstacle();
    } else {
      double leftSpeed = 255;
      double rightSpeed = 255;
      double theta = locT - angleTo(x, y);
      // abs is evil.
      double atheta = theta;
      if (atheta < 0) {
        atheta *= -1.0;
      }

      // Robot slows down as it gets closer to obstacle.
      double dist = getUltraDistance(TRIG_PIN, ECHO_PIN);
      if (obsCheck && dist < 60.0) {
        leftSpeed -= ((60.0 - dist) / 60.0) * (255.0 - MIN_SPEED);
        rightSpeed -= ((60.0 - dist) / 60.0) * (255.0 - MIN_SPEED);
      }

      // Prints error
      Enes100.print("error: ");
      Enes100.println(atheta);

      // Checks there is enough error to correct for.
      if (atheta > 0.01) {
        // Corrects in the correct direction.
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

      // Checks if destination has been reached.
      if (distanceTo(x, y) < .05) {
        Enes100.println("Drived!");
        flag = true;
        leftSpeed = 0;
        rightSpeed = 0;
      }

      // Print speeds
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
  if (locY - 0.333)
    if (locY < 0.66) {
      orient(locY < 0.33 ? 1.57 : -1.57);
      driveFar(locX, 0.33, false);
    } else if (locY < 1.33) {
      orient(locY < 1.0 ? 1.57 : -1.57);
      driveFar(locX, 1, false);
    } else {
      orient(locY < 1.66 ? 1.57 : -1.57);
      driveFar(locX, 1.66, false);
    }

  // Orients self to get ready.
  orient(0);

  Enes100.println("All systems go.");
}

// Drives robot around an obstacle
void avoidObstacle() {
  Enes100.println("Avoiding Obstacle!");
  updateEverything();

  // double newY = 0;
  // Finds what lane the OSV is closest to and goes around obstacle accordingly.
  // All numbers found by magic.
  // if (locY > 1.333) {
  //   orient(-3.14 / 2.0);
  //   updateEverything();
  //   newY = 1;
  //   driveFar(locX, 1 + 0.333, false);
  //   driveFar(locX + .4, 1, false);
  // } else if (locY > 1) {
  //   orient(3.14 / 2.0);
  //   updateEverything();
  //   newY = 1.666;
  //   driveFar(locX, 1.666 - 0.333, false);
  //   driveFar(locX + .4, 1.666, false);
  // } else if (locY > 0.666) {
  //   orient(-3.14 / 2.0);
  //   updateEverything();
  //   newY = 0.333;
  //   driveFar(locX, 0.333 + 0.333, false);
  //   driveFar(locX + .4, 0.333, false);
  // } else {
  //   orient(3.14 / 2.0);
  //   updateEverything();
  //   newY = 1;
  //   driveFar(locX, 1 - 0.333, false);
  //   driveFar(locX + .4, 1, false);
  // }

  // Why write many line when few line do trick?
  double newY = locY > 1.333 || locY < 0.666 ? 1 : locY > 1 ? 1.666 : 0.333;
  double middleY = locY > 1 ? 1.333 : 0.666;
  orient(locY > 1.333 || (locY < 1 && locY > 0.666) ? -1.57 : 1.57);
  driveFar(locX, middleY, false);
  driveFar(locX + 0.4, newY, false);

  // Re-orient forwards.
  orient(0);

  Enes100.println("Obstacle Avoided!");
  driveFar(desX - DESTINATION_BUFFER_DISTANCE, newY, true);
}

// Points robot towards specified angle.
// double t - An angle in radians
void orient(double t) {
  Enes100.print("Orienting to ");
  Enes100.println(t);

  // Create loop to run until interruption.
  bool flag = false;
  while (!flag) {
    updateEverything();

    // Calculate error.
    double theta = locT;
    double error = theta - t;

    // Print error.
    Enes100.print("error: ");
    Enes100.println(error);

    // Check if orientation is correct.
    // abs is evil.
    if (error < 0.01 && error > -0.01) {
      Enes100.println("Oriented!");
      flag = true;
      setMotorSpeed(0, 0);
    } else {
      // Calculate speed to spin at based on error.
      double output = abs(error) * ORIENT_kP + MIN_SPEED_TURN;

      // Limits output to max of 255
      if (output > 255) {
        output = 255;
      }

      // Print output.
      Enes100.print("output: ");
      Enes100.println(output);

      // Spins in correct direction.
      if (error > 0) {
        setMotorSpeed((int)output, (int)-output);
      } else {
        setMotorSpeed((int)-output, (int)output);
      }
    }

    // Delay for reasons.
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
  // Spin motors in correct direction.
  // ternary operators are cool. looks like magic
  frontLeftMotor->run(left < 0 ? BACKWARD : FORWARD);
  backLeftMotor->run(left < 0 ? BACKWARD : FORWARD);
  frontRightMotor->run(right < 0 ? BACKWARD : FORWARD);
  backRightMotor->run(right < 0 ? BACKWARD : FORWARD);

  // Set power of motors. (must be positive)
  backRightMotor->setSpeed(abs(right));
  frontRightMotor->setSpeed(abs(right));
  frontLeftMotor->setSpeed(abs(left));
  backLeftMotor->setSpeed(abs(left));
}

// Returns weight measured by load cell.
double getWeight() {
  scale.set_scale(calibration_factor);
  return scale.get_units(40);
}

// Print out stats every time updateEverything is ran.
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

void Near_Field_Nav() {
  int Mat_Dist[15][2];
  Fill_Array(Mat_Dist);
  orient(find_min(Mat_Dist));
  while (getUltraDistance > 8) {
    setMotorSpeed(255, 255);
  }
  //while(getWeight<crit) {
  // Drop Claw
  // Lift Claw
  //}
  //sendData(getWeight, getMagneto)
}

void Fill_Array(int Mat_Dist[][2]) {
  for (int i = 0; i < 5; i++) {
    orient(locT + (.175));
    Mat_Dist[i][0] = locT;
    Mat_Dist[i][1] = getUltraDistance(TRIG_PIN, ECHO_PIN);
  }
  for (int i = 5; i < 15; i++) {
    orient(locT - (.175));
    Mat_Dist[i][0] = locT;
    Mat_Dist[i][1] = getUltraDistance(TRIG_PIN, ECHO_PIN);
  }
}

int find_min(int Mat_Dist[][2]) {
  int min_val;
  int min_loc;
  for (int i = 0; i < 15; i++) {
    if (Mat_Dist[i][1] < min_val) {
      min_val = Mat_Dist[i][1];
      min_loc = Mat_Dist[i][0];
    }
  }
}
