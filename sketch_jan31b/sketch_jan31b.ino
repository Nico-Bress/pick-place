#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Definizione del tipo di driver: 1 = DRIVER (usa STEP e DIR)
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 5  // Opzionale
double i = 0;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int MOTOR_MAX_SPEED = (3200/60)*50 ; //3200 = 1 giro al secondo, (3200/60) = 1 rpm
const int giri = (3300*2) ; //3300 = 1 giro


unsigned long previousMillis = 0;   // Memorizza il tempo dell'ultima accensione
const long interval = 3000;          // Intervallo di lampeggio (3 secondo)


void setup() {
  SerialPort.begin(115200);
  SerialPort.println("Starting...");
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Attiva il driver (LOW = attivo)

  stepper.setMaxSpeed(MOTOR_MAX_SPEED);  // Velocità massima (passi/sec)
  stepper.setAcceleration(1000); // Accelerazione (passi/sec^2)
}

void loop() {
  while(i<=(giri)) { //3400 1 GIRO (175000)
    stepper.setSpeed(MOTOR_MAX_SPEED);
    stepper.runSpeed();
    //stepper.step(3200);
    SerialPort.println(i);
    i++;
  }
  i=0;
  stepper.setSpeed(0);
  stepper.runSpeed();
  while(i<=(giri)){
    stepper.setSpeed(-MOTOR_MAX_SPEED);
    stepper.runSpeed();
    //stepper.step(3200);
    SerialPort.println(i);
    i++;
  }
  i=0;
}