#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <AccelStepper.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Pin definitions
#define LED_PIN LED_BUILTIN
#define TOF1_XSHUT_PIN A1
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 5

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ToF sensors
VL53L4CD tof1(&DEV_I2C, TOF1_XSHUT_PIN);

// Positions
enum Position { A, B };
Position currentPosition;

// Constants for distances
const int POSITION_A_DISTANCE = 15; // mm
const int POSITION_B_DISTANCE = 15; // mm
const int RANGE_TOLERANCE = 5;      // mm
const int SLOWDOWN_THRESHOLD = 100; // mm
const int MOTOR_MAX_SPEED = 3200;    // 3200 = giro al secondo
const int MOTOR_SLOW_SPEED = 800;    // 600 = 1/4 di giro al secondo

// Function prototypes
void initToF(VL53L4CD &sensor, uint8_t xshutPin);
void moveMotor(Position target);
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

  // Initialize ToF sensors
  initToF(tof1, TOF1_XSHUT_PIN);

  // Initialize motor
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Attiva il driver (LOW = attivo)

  stepper.setMaxSpeed(MOTOR_MAX_SPEED);  // Velocità massima (passi/sec)
  stepper.setAcceleration(1000); // Accelerazione (passi/sec^2)

  // Detect initial position
  currentPosition = detectInitialPosition();

  // Move to position A if not already there
  if (currentPosition != A) {
    SerialPort.println("Moving to home position (A)...");
    moveMotor(A);
    currentPosition = A;
  }

  SerialPort.println("System initialized and set to home position.");
}

// Loop
void loop() {
  // Check if GO button is pressed (assuming you still have a GO button)
  if (digitalRead(3) == LOW) { // Replace 3 with your GO button pin if different
    SerialPort.println("Starting Pick & Place operation...");

    // Move to position B
    moveMotor(B);

    // Wait for operation completion
    delay(500);

    // Move to position A
    moveMotor(A);

    // Wait for operation completion
    delay(500);

    // Move to position B
    moveMotor(B);

    // Wait for operation completion
    delay(500);

    SerialPort.println("Pick & Place operation completed.");
  }
}

// Function to initialize ToF sensors
void initToF(VL53L4CD &sensor, uint8_t xshutPin) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(10);
  digitalWrite(xshutPin, HIGH);
  delay(10);

  sensor.begin();
  sensor.VL53L4CD_Off();
  sensor.InitSensor();
  sensor.VL53L4CD_SetRangeTiming(200, 0);
  sensor.VL53L4CD_StartRanging();
}

// Function to detect initial position (using only tof1 now)
Position detectInitialPosition() {
  int distanceToA = readToF(tof1);

  if (abs(distanceToA - POSITION_A_DISTANCE) <= RANGE_TOLERANCE) {
    SerialPort.println("Initial position detected: A");
    return A;
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
  VL53L4CD &targetSensor = (target == A) ? tof1 : tof1; // Use tof1 for both now

  while (true) {
    int currentDistance = readToF(targetSensor);

    if (currentDistance <= targetDistance + RANGE_TOLERANCE && currentDistance >= targetDistance - RANGE_TOLERANCE) {
      stepper.setSpeed(0); // Stop the motor
      stepper.runSpeed();  // Esegue l'ultima velocità impostata
      currentPosition = target;
      SerialPort.println("Target position reached.");
      break;
    }

    if (currentDistance < SLOWDOWN_THRESHOLD) {
      stepper.setSpeed(MOTOR_SLOW_SPEED);
    } else {
      stepper.setSpeed(MOTOR_MAX_SPEED);
    }

    // Move the motor step by step
    stepper.runSpeed(); // Esegue lo step del motore
  }
}

// Function to read distance from a ToF sensor
int readToF(VL53L4CD &sensor) {
  VL53L4CD_Result_t results;
  sensor.VL53L4CD_GetResult(&results);
  return results.distance_mm;
}