#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <AccelStepper.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Pin definitions
#define LED_PIN LED_BUILTIN
#define TOF1_XSHUT_PIN A1
#define TOF2_XSHUT_PIN A2
#define VACUUM_VALVE_PIN 10
#define CYLINDER_VALVE_PIN 11
#define EMERGENCY_SWITCH_PIN 8
#define GO_BUTTON_PIN 7
#define HOME_BUTTON_PIN 6
#define STOP_BUTTON_PIN 5 // New pin for stop button
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 5

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ToF sensors
VL53L4CD tof1(&DEV_I2C, TOF1_XSHUT_PIN);
VL53L4CD tof2(&DEV_I2C, TOF2_XSHUT_PIN);

// Positions
enum Position { A, B };
Position currentPosition;

// Constants for distances
const int POSITION_A_DISTANCE = 15; // mm
const int POSITION_B_DISTANCE = 15; // mm
const int RANGE_TOLERANCE = 5;      // mm
const int SLOWDOWN_THRESHOLD = 100; // mm
const int MOTOR_MAX_SPEED = 3200*50;    // 3200 = giro al secondo
const int MOTOR_SLOW_SPEED = 800*50;    // 600 = 1/4 di giro al secondo

// Function prototypes
void initToF(VL53L4CD &sensor, uint8_t xshutPin, uint8_t newAddress);
void moveMotor(Position target);
void controlVacuum(bool state);
void controlCylinder(bool state);
bool checkEmergencyStop();
int readToF(VL53L4CD &sensor);
Position detectInitialPosition();

// Setup
void setup() {
  // Initialize serial communication
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus
  DEV_I2C.begin();

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);

  // Initialize buttons and emergency switch
  pinMode(EMERGENCY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(GO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(HOME_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP); // Initialize stop button

  // Initialize valves
  pinMode(VACUUM_VALVE_PIN, OUTPUT);
  pinMode(CYLINDER_VALVE_PIN, OUTPUT);

  // Initialize ToF sensors
  initToF(tof1, TOF1_XSHUT_PIN, 0x52);
  initToF(tof2, TOF2_XSHUT_PIN, 0x54);

  // Initialize motor
  stepper.setSpeed(MOTOR_MAX_SPEED);

  // Detect initial position
  currentPosition = detectInitialPosition();

  // Move to position A if not already there
  if (currentPosition != A) {
    SerialPort.println("Moving to home position (A)...");
    moveMotor(A);
    currentPosition = A;
  }

  // Reset system to home position
  controlVacuum(false);
  controlCylinder(false);
  SerialPort.println("System initialized and set to home position.");
}
// 
//
//
//looooooop
//
//
//

void loop() {
  // Check for emergency stop
  if (checkEmergencyStop()) {
    SerialPort.println("EMERGENCY STOP ACTIVATED!");
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  // Check for stop button
  if (digitalRead(STOP_BUTTON_PIN) == LOW) { 
    SerialPort.println("Stopping and returning to home position...");
    stepper.setSpeed(0); // Stop the motor immediately
    moveMotor(A); // Move to home position
    controlVacuum(false);
    controlCylinder(false);
    currentPosition = A;
  }

  // Check if HOME button is pressed
  if (digitalRead(HOME_BUTTON_PIN) == LOW) {
    SerialPort.println("Returning to home position...");
    moveMotor(A);
    controlVacuum(false);
    controlCylinder(false);
    currentPosition = A;
  }

  // Check if GO button is pressed
  if (digitalRead(GO_BUTTON_PIN) == LOW) {
    SerialPort.println("Starting Pick & Place operation...");

    // Move to position B, release vacuum, lift cylinder
    moveMotor(B);
    controlVacuum(false);
    controlCylinder(true);

    // Wait for operation completion
    delay(500);

    // Move to position A, activate vacuum, lower cylinder
    moveMotor(A);
    controlCylinder(false);
    controlVacuum(true);

    // Wait for operation completion
    delay(500);

    // Move to position B, release vacuum, lift cylinder
    moveMotor(B);
    controlCylinder(true);
    controlVacuum(false);

    // Wait for operation completion
    delay(500);

    SerialPort.println("Pick & Place operation completed.");
  }
}

// 
//
//
//fine looooooop
//
//
//
// Function to initialize ToF sensors
void initToF(VL53L4CD &sensor, uint8_t xshutPin, uint8_t newAddress) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(10);
  digitalWrite(xshutPin, HIGH);
  delay(10);

  sensor.begin();
  sensor.VL53L4CD_Off();
  sensor.InitSensor();
  sensor.VL53L4CD_SetI2CAddress(newAddress);
  sensor.VL53L4CD_SetRangeTiming(200, 0);
  sensor.VL53L4CD_StartRanging();
}

// Function to detect initial position
Position detectInitialPosition() {
  int distanceToA = readToF(tof1);
  int distanceToB = readToF(tof2);

  if (abs(distanceToA - POSITION_A_DISTANCE) <= RANGE_TOLERANCE) {
    SerialPort.println("Initial position detected: A");
    return A;
  } else if (abs(distanceToB - POSITION_B_DISTANCE) <= RANGE_TOLERANCE) {
    SerialPort.println("Initial position detected: B");
    return B;
  } else {
    SerialPort.println("Initial position unknown, assuming position A.");
    return A;
  }
}

// Function to move the motor to a target position
void moveMotor(Position target) {
  SerialPort.print("Moving from position ");
  SerialPort.print(currentPosition);
  SerialPort.print(" to position ");
  SerialPort.println(target);

  int targetDistance = (target == A) ? POSITION_A_DISTANCE : POSITION_B_DISTANCE;
  VL53L4CD &targetSensor = (target == A) ? tof1 : tof2;
  VL53L4CD &oppositeSensor = (target == A) ? tof2 : tof1;

  while (true) {
    int currentDistance = readToF(targetSensor);

    if (currentDistance <= targetDistance + RANGE_TOLERANCE && currentDistance >= targetDistance - RANGE_TOLERANCE) {
      stepper.setSpeed(0); // Stop the motor
      currentPosition = target;
      SerialPort.println("Target position reached.");
      break;
    }

    if (currentDistance < SLOWDOWN_THRESHOLD || readToF(oppositeSensor) < SLOWDOWN_THRESHOLD) {
      stepper.setSpeed(MOTOR_SLOW_SPEED);
    } else {
      stepper.setSpeed(MOTOR_MAX_SPEED);
    }

    // Move the motor step by step
    stepper.step((target == A) ? -1 : 1);
  }
}
/*
// Function to control the vacuum valve
void controlVacuum(bool state) {
  digitalWrite(VACUUM_VALVE_PIN, state ? HIGH : LOW);
  SerialPort.print("Vacuum valve ");
  SerialPort.println(state ? "ON" : "OFF");
}

// Function to control the cylinder valve
void controlCylinder(bool state) {
  digitalWrite(CYLINDER_VALVE_PIN, state ? HIGH : LOW);
  SerialPort.print("Cylinder valve ");
  SerialPort.println(state ? "UP" : "DOWN");
}

// Function to check for emergency stop
bool checkEmergencyStop() {
  return digitalRead(EMERGENCY_SWITCH_PIN) == LOW;
}
*/
// Function to read distance from a ToF sensor
int readToF(VL53L4CD &sensor) {
  VL53L4CD_Result_t results;
  sensor.VL53L4CD_GetResult(&results);
  return results.distance_mm;
}

