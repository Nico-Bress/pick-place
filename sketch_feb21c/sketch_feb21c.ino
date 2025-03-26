#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

#define DEV_I2C Wire
#define SerialPort Serial

// Pin definitions
#define TOF1_XSHUT_PIN A1
#define TOF2_XSHUT_PIN A2

// ToF sensors
VL53L4CD tof1(&DEV_I2C, TOF1_XSHUT_PIN);
VL53L4CD tof2(&DEV_I2C, TOF2_XSHUT_PIN);


// Function prototypes
bool initToF(VL53L4CD &sensor, uint8_t xshutPin, uint8_t newAddress, const char *sensorName);
int readToF(VL53L4CD &sensor);

void setup() {
  // Initialize serial communication
  SerialPort.begin(115200);
  while (!SerialPort); // Wait for serial port to connect (solo per schede con USB nativa)

  // Initialize I2C bus
  DEV_I2C.begin();

  // Initialize ToF sensors with connection check
  bool tof1Connected = initToF(tof1, TOF1_XSHUT_PIN, 0x52, "Sensore 1");
  bool tof2Connected = initToF(tof2, TOF2_XSHUT_PIN, 0x54, "Sensore 2");

  // Stop if any sensor is not connected
  if (!tof1Connected || !tof2Connected) {
    SerialPort.println("Errore: Uno o più sensori non sono connessi correttamente. Verifica i collegamenti.");
    while (true); // Blocca il programma
  }

  SerialPort.println("Sensori ToF inizializzati correttamente!");
}

void loop() {
  // Leggi la distanza dal primo sensore
  int distance1 = readToF(tof1);
  if (distance1 >= 0) {
    SerialPort.print("Distanza Sensore 1: ");
    SerialPort.print(distance1);
    SerialPort.print(" mm");
  } else {
    SerialPort.print("Errore lettura Sensore 1");
  }

  // Leggi la distanza dal secondo sensore
  int distance2 = readToF(tof2);
  if (distance2 >= 0) {
    SerialPort.print(" | Distanza Sensore 2: ");
    SerialPort.print(distance2);
    SerialPort.println(" mm");
  } else {
    SerialPort.println(" | Errore lettura Sensore 2");
  }

  // Aspetta un po' prima della prossima lettura
  delay(100); // Riduci il delay per una frequenza di aggiornamento più alta
}

// Funzione per inizializzare un sensore ToF e verificare la connessione
bool initToF(VL53L4CD &sensor, uint8_t xshutPin, uint8_t newAddress, const char *sensorName) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW); // Spegni il sensore
  delay(50); // Aumenta il delay per garantire un riavvio corretto
  digitalWrite(xshutPin, HIGH); // Riavvia il sensore
  delay(50);

  // Verifica se il sensore è connesso
  if (!sensor.begin()) {
    SerialPort.print(sensorName);
    SerialPort.println(" non trovato! Verifica i collegamenti.");
    return false; // Sensore non connesso
  }

  sensor.VL53L4CD_Off();
  sensor.InitSensor();
  sensor.VL53L4CD_SetI2CAddress(newAddress); // Cambia l'indirizzo I2C
  sensor.VL53L4CD_SetRangeTiming(200, 0); // Configura il timing di misurazione
  sensor.VL53L4CD_StartRanging(); // Avvia la misurazione

  SerialPort.print(sensorName);
  SerialPort.println(" connesso correttamente.");
  return true; // Sensore connesso
}

// Funzione per leggere la distanza da un sensore ToF
int readToF(VL53L4CD &sensor) {
  VL53L4CD_Result_t results;
  if (sensor.VL53L4CD_GetResult(&results)) { // Leggi il risultato
    return results.distance_mm; // Restituisci la distanza in millimetri
  } else {
    return -1; // Restituisci -1 in caso di errore
  }
}