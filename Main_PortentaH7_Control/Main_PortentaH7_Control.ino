#include <WiFi.h>
#include "Arduino.h"
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include <Adafruit_PWMServoDriver.h>


// Initialize PWM driver, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ===== SD Configuration =======
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");
bool sdCardInitialized = false;
static String filename = ""; // global

// ========== Motor Configuration via PCA9685 ==========
// DRV8833 #1 - Vertical Motors
#define LEFT_AIN1_PWM   0  // CH0 PCA9685
#define LEFT_AIN2_PWM   1  // CH1 PCA9685
#define LEFT_BIN1_PWM   2  // CH2 PCA9685
#define LEFT_BIN2_PWM   3  // CH3 PCA9685
#define LEFT_SLP        D0

// DRV8833 #2 - Left Motors
#define VERT_AIN1_PWM   4  // CH4 PCA9685
#define VERT_AIN2_PWM   5  // CH5 PCA9685
#define VERT_BIN1_PWM   6  // CH6 PCA9685
#define VERT_BIN2_PWM   7  // CH7 PCA9685
#define VERT_SLP        D1

// DRV8833 #3 - Right Motors
#define RIGHT_AIN1_PWM  8  // CH8 PCA9685
#define RIGHT_AIN2_PWM  9  // CH9 PCA9685
#define RIGHT_BIN1_PWM 10  // CH10 PCA9685
#define RIGHT_BIN2_PWM 11  // CH11 PCA9685
#define RIGHT_SLP      D2 

// PWM Settings
const int PWM_MAX = 4095;     // 12-bit resolution (0-4095)

// ========== Network Configuration ==========
// WiFi settings
const char* ssid = "CERN";
const char* password = "";
WiFiUDP Udp;

// NTP Settings (VERY IMPORNTANT: Your computer must have a static IP address and a permanent Internet connection.)
const long utcOffsetWinter = 3600; // UTC+1 (CET)
const long utcOffsetSummer = 7200; // UTC+2 (CEST)
const char* NTP_SERVER = "pool.ntp.org";
bool ntpInitialized = false;
WiFiUDP ntpUdp; 
NTPClient ntpClient(ntpUdp, NTP_SERVER, utcOffsetWinter);
static unsigned long lastNTPUpdate = 0;
const unsigned long ntpInterval = 60000; // 60 seconds of NTP update

// UDP Settings
const unsigned int localPort = 2390;    // Local port to listen on
const unsigned int remotePort = 2391;   // Port to send responses to

// IPAddress remoteIP(128,141,178,110); // 190 R002 PC
IPAddress remoteIP(194,12,159,94);     // Laptop

#define UDP_PACKET_SIZE 64              // for the joystick packets
char packetBuffer[256];                 // Buffer to hold incoming packets for UDP communication
unsigned long lastJoystickPacket = 0;

// ========== System State ==========
enum ControlMode {
  MODE_SAFETY,        // Default safety mode (no thrust)
  MODE_MANUAL,        // Manual control (raw user input)
  MODE_ALTITUDE_HOLD, // Auto-maintain altitude
  MODE_HEADING_HOLD,  // Attitude stabilization (auto-level)
  MODE_HOVER,         // Manatin x y z yaw
  MODE_WAYPOINT       // Follow waypoints
};

ControlMode currentControlMode = MODE_SAFETY;

// ========== Manual Mode Settings ==========
float manualControlValues[3] = {0}; // Stores surge, yaw, heave values

// Global variables for motor thrust
float leftThrust = 0.0f;
float rightThrust = 0.0f;
float verticalThrust = 0.0f;

// ====================================
// --- Configuration voltage sensor ---
// ====================================
const int voltagePin = A0;       // Pin analogico collegato al sensore
const float scaleFactor = 5.0;   // Fattore di scala del partitore 
const float adcReference = 3.3;  // Tensione di riferimento ADC
const int adcMax = 4095;         // Risoluzione ADC (12 bit per Portenta H7)

struct BatteryStatus {
    float voltage = 0.0;
    float percent = 0.0;
};

BatteryStatus batteryStatus;  // ora voltage e percent partono già da 0.0
static unsigned long lastBatteryRead = 0;
const unsigned long batteryInterval = 500;  // every 0.5 seconds


// =====6 DoF vector from QTM ========
struct MotionCaptureData {
  float posX = NAN;
  float posY = NAN;
  float posZ = NAN;
  float roll = NAN;
  float pitch = NAN;
  float yaw = NAN;
  bool mocapValid = false;
};

MotionCaptureData mocapData;
static unsigned long lastValidMocapTime = 0;

// ===== Waypoint mission settings =====
struct Waypoint {
  float x;
  float y;
  float z;
  float yaw;
};

Waypoint target = {0.0f, 0.0f, 0.0f, 0.0f};
float distToTarget = 0.0f;

const int MAX_WAYPOINTS = 20;
Waypoint waypointQueue[MAX_WAYPOINTS];

int waypointCount = 0;
int currentWaypointIndex = 0;

bool waypointReached = false;
bool missionActive = false;

// ========== ToF Sensor =================
VL53L4CX sensor_vl53l4cx_sat(&Wire, A1);  // Address 0x10 (A1 = XSHUT pin)
float tofAltitude_m = 0.0;
bool tofAvailable = false;

//=========== MAIN LOOP ============
void setup() {
  Serial.begin(115200);
  Wire.begin();  // <--- Assicurati di inizializzare I²C prima di usare il sensore ToF

   // Connect to network
  initializeWiFi();    // Connessione WiFi
  
  //  Start UDP server
  if (Udp.begin(localPort)) {
    log("[INFO]: UDP server started on port " + String(localPort));
  } else {
    log("[ERROR]: Failed to start UDP server!");
  }

  // Initialize NTP
  ntpClient.begin();   // Inizializza NTP solo dopo la connessione WiFi
  ntpClient.update();  // Forza un aggiornamento immediato
  ntpInitialized = true;
  log("[INFO]: NTP client initialized");
  printCurrentDateTime(); // Stampa la data e l'ora corrente

  // Voltage sensor reader
  analogReadResolution(12); 

  // Initialize SD card  
  initSDCard(); 

  // Initialize control systems
  initializePWM();
 
  // Setup PID controllers
  setupVerticalControl();
  // setupHeadingPID();  
  // setupVelocityPID();

  // Inizialize ToF sensor 
  initializeToF(); 

  Serial.println("Portenta H7 Sensor Data Receiver Ready");
}

void loop() {
  // Aggiornamento NTP periodico
  if (millis() - lastNTPUpdate > ntpInterval) {
      ntpClient.update();
      lastNTPUpdate = millis();
  }

  // Aggiornamento batteria
  if (millis() - lastBatteryRead >= batteryInterval) {
      lastBatteryRead = millis();
      readBattery();
  }

  handleUDPCommunication();    // 2. Handle UDP communication

  handleControlMode();         // 4. Handle current control mode
  // safetyCheckLoop();        // 5. Centralized safety checks
  // handlePeriodicTasks();    // 6. Periodic tasks (MODE, IP, TIME, Battery Level)

  // SENSORS 
  readVL53L4CX();
  
  logVerticalControl(); 
  
  logSensorData();          // 7. SD Log 

  // Debug print
  // printMocapData();

}





