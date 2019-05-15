#include <Adafruit_MotorShield.h>
#include <DFRobot_QMC5883.h>
#include <Enes100.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "HX711.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

/*
* Setup Constants
*/
// Debug Prints
#define DEBUG false
// variable to store the servo position
int pos = 0;
// Minimum speed required to turn
#define MIN_SPEED_TURN 45.0
// Distance in centimeters that triggers obstacle
#define OBSTACLE_TRIGGER_DISTANCE 11.5
// Distance we want the far navigation to end up from the mission site
#define DESTINATION_BUFFER_DISTANCE 0.25
// Minimum speed OSV requires to move forwards
#define MIN_SPEED 30.0
// Proportional factor for driveFar corrections
#define DRIVE_FAR_kP 255.0 * 5.0 / 3.14
// Proportional factor for orient corrections
#define ORIENT_kP ((255.0 - MIN_SPEED_TURN) * 2.0 / 3.14)
// Distance away to approach
#define APPROACH_DIST 0.5

// Trigger pin of ultrasonic sensor
#define TRIG_PIN 12
// Echo pin of ultrasonic sensor
#define ECHO_PIN 11

// Load cell pins
#define DOUT 3
#define CLK 2

// Servo pin
#define SERVO_PIN 9

// Limit switch pins
#define LOWER_LIMIT_SWITCH 8
#define UPPER_LIMIT_SWITCH 13

// Makes getting location values easier
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

// Create Servo Object
Servo myservo;

// Create Magneto Object
DFRobot_QMC5883 compass;

// Magic
float calibration_factor = -242;

void setup() {
  // Initialize OSV
  bootUp();
  Enes100.println("OSV Initialized.");

  // Make sure arm is lifted all the way
  attachServo();
  liftArm();
  unAttachServo();

  // Moves OSV to one of the three colunms.
  startUp();

  // Drive to the target to buffer distance.
  driveFar(Enes100.destination.x - DESTINATION_BUFFER_DISTANCE, locY, true);

  // Move to be above or below target by APPROACH_DIST meters
  double targetY = getRow(desY) == 1 ? desY + APPROACH_DIST : desY - APPROACH_DIST;
  orient((locY - targetY) > 0 ? -1.57 : 1.57);
  driveFar(locX, targetY, false);
  orient(0);

  // Drive to be aligned with target on y-axis.
  driveClose(desX - locX + 0.06);

  // Point towards debris
  orient((locY - desY) > 0 ? -1.57 : 1.57);

  // Back up if too close
  if (distanceTo(desX, desY) < .50) {
    driveClose(distanceTo(desX, desY) - 0.5);
  }

  // ReAttach Servo
  attachServo();

  // Lower Arm
  Enes100.println("Lowering Arm");
  lowerArm();
  Enes100.println("Arm Lowered");

  // UnAttach Servo
  unAttachServo();

  // Measure magneto baseline
  Enes100.println("Measuring Baseline");
  delay(1000);
  double baseline = magneto();
  Enes100.print("Baseline: ");
  Enes100.println(baseline);

  // Drive up close
  driveClose(distanceTo(desX, desY) - .13);
  updateEverything();

  // Take magneto measuring now that debris is loaded
  double measurement = magneto();
  Enes100.println(measurement);

  // Reattach Servo
  attachServo();

  // Lift arm
  Enes100.println("Lifting Arm");
  liftArm();
  Enes100.println("Arm Lifted");

  // Drop arm just a little bit.
  delay(1000);

  // Return if debris is steel or copper
  if (steelCheck(baseline, measurement)) {
    Enes100.mission(STEEL);
  } else {
    Enes100.mission(COPPER);
  }

  // Return measured weight
  delay(2000);
  Enes100.mission(getWeight(20));
}

// All Initialization Code
void bootUp() {
  // Limit Switch Setup
  pinMode(LOWER_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(UPPER_LIMIT_SWITCH, INPUT_PULLUP);

  delay(10000);

  // Wait for connection to vision system.
  // Team Name, Mission Type, Marker ID, RX Pin, TX Pin
  while (!Enes100.begin("Weigh to go", DEBRIS, 9, 7, 6)) {
  }

  // shrug?
  delay(500);

  Enes100.println("Connected!");

  // print out destination location
  Enes100.print("Destination is at (");
  Enes100.print(Enes100.destination.x);
  Enes100.print(", ");
  Enes100.print(Enes100.destination.y);
  Enes100.println(")");

  // Set pin modes of ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Start Motor Controller.
  AFMS.begin();  // create with the default frequency 1.6KHz

  // Initialize Initialize QMC5883
  while (!compass.begin()) {
    Enes100.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  if (compass.isHMC()) {
    Enes100.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  } else if (compass.isQMC()) {
    Enes100.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
    Enes100.println("Initialized QMC5883");
  }

  // Prints location data for 2.5 seconds
  long start = millis();
  while (millis() - start < 2000) {
    updateEverything();
  }
  Enes100.println("Initializing scale");
  // Scale initilizaiton.
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.tare();  //Reset the scale to 0
  Enes100.println("Scale Initialized");
}

// Drives to a point on the field with obstacle avoidance if obsCheck = true.
void driveFar(double x, double y, bool obsCheck) {
  Enes100.print("Driving to ");
  Enes100.print(x);
  Enes100.print(", ");
  Enes100.println(y);
  double oldDist = getUltraDistance(TRIG_PIN, ECHO_PIN);

  // Run a loop until interruption
  bool flag = false;
  while (!flag) {
    updateEverything();

    // Check for obstacle.
    if (obsCheck && obstacle()) {
      Enes100.println("Obstacle Found!");
      flag = true;
      setMotorSpeed(0, 0);
      avoidObstacle();
    } else {
      double leftSpeed = 255;
      double rightSpeed = 255;
      double theta = locT - angleTo(x, y);

      // Removes extraneous values
      double dist = getUltraDistance(TRIG_PIN, ECHO_PIN);
      if ((dist - oldDist) > 80.0 || (oldDist - dist) > 80.0) {
        dist = oldDist;
        oldDist = getUltraDistance(TRIG_PIN, ECHO_PIN);
      }

      // Robot slows down as it gets closer to obstacle.
      if (obsCheck && dist < 50.0) {
        leftSpeed -= ((50.0 - dist) / 50.0) * (255.0 - MIN_SPEED);
        rightSpeed -= ((50.0 - dist) / 50.0) * (255.0 - MIN_SPEED);
      }

      // Prints error
      if (DEBUG) {
        Enes100.print("error: ");
        Enes100.println(theta);
      }

      // Checks there is enough error to correct for.
      if (fabs(theta) > 0.01) {
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
      if (distanceTo(x, y) < .1) {
        Enes100.println("Drived!");
        flag = true;
        leftSpeed = 0;
        rightSpeed = 0;
      }

      // Print speeds
      if (DEBUG) {
        Enes100.print("left: ");
        Enes100.print(leftSpeed);
        Enes100.print(", right: ");
        Enes100.println(rightSpeed);
      }
      setMotorSpeed(leftSpeed, rightSpeed);
    }
  }
}

// Drives slowly a distance in m
// if dist>0 goes forwards, if dist<0 goes backwards
void driveClose(double dist) {
  boolean negative = false;
  if (dist < 0) {
    negative = true;
    dist = -dist;
  }
  Enes100.print("Driving ");
  Enes100.print(dist);
  Enes100.println("m");
  updateEverything();
  double startX = locX;
  double startY = locY;

  // Run a loop until interruption
  bool flag = false;
  while (!flag) {
    updateEverything();
    double leftSpeed = negative ? -50 : 50;
    double rightSpeed = negative ? -50 : 50;

    // Checks if destination has been reached.
    if (distanceTo(startX, startY) >= dist) {
      Enes100.println("Drived!");
      flag = true;
      leftSpeed = 0;
      rightSpeed = 0;
    }
    setMotorSpeed(leftSpeed, rightSpeed);
  }
}

// Run first to get robot aligned to a row.
void startUp() {
  Enes100.println("Initiating Launch Sequence.");
  updateEverything();

  // Depending on row, drives to center of nearest.
  // Tolerance of .2 meters from center of row to activate
  if (!(fabs(locY - 0.333) < 0.2 || fabs(locY - 1) < 0.2 || fabs(locY - 1.666) < 0.2)) {
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
  }

  // Orients self to get ready.
  orient(0);

  Enes100.println("All systems go.");
}

// Drives robot around an obstacle
void avoidObstacle() {
  Enes100.println("Avoiding Obstacle!");
  updateEverything();

  // Why write many line when few line do trick?
  double newY = locY > 1.333 || locY < 0.666 ? 1 : locY > 1 ? 1.666 : 0.333;
  double middleY = locY > 1 ? 1.333 : 0.666;
  orient(locY > 1.333 || (locY < 1 && locY > 0.666) ? -1.57 : 1.57);
  driveFar(locX, newY, false);

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
    if (DEBUG) {
      Enes100.print("error: ");
      Enes100.println(error);
    }

    // Check if orientation is correct.
    // abs is evil.
    if (error < 0.01 && error > -0.01) {
      Enes100.println("Oriented!");
      flag = true;
      setMotorSpeed(0, 0);
    } else {
      // Calculate speed to spin at based on error.
      double output = fabs(error) * ORIENT_kP + MIN_SPEED_TURN;

      // Limits output to max of 255
      if (output > 255) {
        output = 255;
      }

      // Print output.
      if (DEBUG) {
        Enes100.print("output: ");
        Enes100.println(output);
      }

      // Spins in correct direction.
      if (error > 0) {
        setMotorSpeed((int)output, (int)-output);
      } else {
        setMotorSpeed((int)-output, (int)output);
      }
    }
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
// int samples - how many samples are averaged together for the reading
double getWeight(int samples) {
  scale.set_scale(calibration_factor);
  return scale.get_units(samples);
}

// Print out stats every time updateEverything is ran.
void printStats() {
  if (DEBUG) {
    Enes100.print("Location: ");
    Enes100.print(locX);
    Enes100.print(", ");
    Enes100.print(locY);
    Enes100.print(", ");
    Enes100.println(locT);
    Enes100.print("Sensors: ");
    Enes100.print(getWeight(1));
    Enes100.print("    ");
    Enes100.println(getUltraDistance(TRIG_PIN, ECHO_PIN));
  }
}

// Returns reading from magnetometers Y axis
double magneto() {
  Vector mag = compass.readRaw();
  delay(500);
  return mag.YAxis;
}

// Given a baseline reading and test reading, returns true if material is steel, false if otherwise
boolean steelCheck(double baseline, double test) {
  Enes100.print("Delta: ");
  Enes100.println(baseline - test);
  return fabs(baseline - test) > 1500;
}

void lowerArm() {
  for (pos = 91; pos <= 95; pos += 1) {  //Accelerates Servo in 50ms increments
    myservo.write(pos);
    delay(50);
  }

  //While loop checks it button is activated
  int sensorVal = digitalRead(LOWER_LIMIT_SWITCH);
  while (sensorVal == HIGH) {
    sensorVal = digitalRead(LOWER_LIMIT_SWITCH);
  }

  for (pos = 95; pos >= 91; pos -= 1) {
    myservo.write(pos);
    delay(20);
  }
  myservo.writeMicroseconds(1500);
}

void liftArm() {
  //Accelerates to full CW
  for (pos = 91; pos >= 81; pos -= 1) {
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(50);           // waits 15ms for the servo to reach the position
  }

  int sensorVal2 = digitalRead(UPPER_LIMIT_SWITCH);
  while (sensorVal2 == HIGH) {
    sensorVal2 = digitalRead(UPPER_LIMIT_SWITCH);
    if (sensorVal2 == LOW) {
      for (pos = 81; pos <= 87; pos += 1) {
        myservo.write(pos);  // tell servo to go to position in variable 'pos'
        delay(10);           // waits 10ms for the servo to reach the position
      }
      break;  //Break from loop for safe measure
    }
  }
}

// Attaches Servo
void attachServo() {
  Enes100.println("Attaching Servo");
  myservo.attach(SERVO_PIN);
  myservo.write(91);
  Enes100.println("Servo Attached");
}

// Unattaches Servo
void unAttachServo() {
  Enes100.println("Unattaching Servo");
  myservo.write(91);
  myservo.attach(15);
  Enes100.println("Servo Unattached");
}

// Gives the angle relative to the horizontal from OSV to target.
double angleTo(double x, double y) {
  return atan2(y - locY, x - locX);
}

// Returns the distance from the OSV to the coordinate passed in
double distanceTo(double x, double y) {
  return sqrt(sq(locX - x) + sq(locY - y));
}

// Anything that needs to be ran every tick goes here
void updateEverything() {
  while (!updateLocation())
    ;
  // Make sure location given makes sense
  while (!((locY < 2 && locY > 0) && (locX < 4 && locX > 0) && (locT > -3.2 && locT < 3.2))) {
    Enes100.println("Bad location");
    updateLocation();
  }
  printStats();
}

// Returns the row that a y coordinate is in if the field were split into 3 rows.
int getRow(double y) {
  return y < 0.666 ? 1 : (y < 1.333 ? 2 : 3);
}