#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <MPU9250.h>  // For accelerometer/gyroscope
#include <TinyGPS++.h> // For GPS speed
#include <RTClib.h>
#include <SD.h>

// Hardware Pins
#define GPS_RX 16
#define GPS_TX 17
#define BUZZER_PIN 5
#define GSM_RST 4
#define SD_CS 13
#define MPU_INT 15
#define LED_PIN 2
#define BATT_PIN 36

// Thresholds (adjust based on your testing.Sijui unataka gani)
#define CRASH_IMPACT_THRESHOLD 3.0  // 3G force
#define SUDDEN_STOP_THRESHOLD 0.5   // 0.5 sec to stop from speed
#define ROLLOVER_ANGLE 90.0         // 90 degrees tilt

// Objects
MPU9250 mpu;               // 
TinyGPSPlus gps;           // 
RTC_DS3231 rtc;            // 
File dataFile;             // 
HardwareSerial ss(1);      // 
HardwareSerial sim800(2);  // 

// Variables
float lastSpeed = 0.0;                      
unsigned long lastSpeedUpdateTime = 0;      
bool accidentConfirmed = false;             

float currentAccelX, currentAccelY, currentAccelZ;  
float accelMagnitude;                               

float rollAngle = 0.0;         
float pitchAngle = 0.0;

String timestamp = "";         
String location = "";          

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU9250
  if (!mpu.begin()) {
    while (1);
  }

  // Initialize GPS (UART1)
  ss.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Initialize SIM800L (UART2)
  sim800.begin(9600, SERIAL_8N1, 26, 27);

  // Initialize RTC
  rtc.begin();

  // Initialize SD card
  SD.begin();

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  // 1. Read Sensor Data
  mpu.readSensor();
  float ax = mpu.getAccelX_mss() / 9.81;  //
  float ay = mpu.getAccelY_mss() / 9.81;
  float az = mpu.getAccelZ_mss() / 9.81;
  float tiltAngle = atan2(ay, az) * 180.0 / PI;  //

  // 2. Read GPS Speed (if available)
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.speed.isValid()) {
        lastSpeed = gps.speed.kmph();
        lastSpeedUpdateTime = millis();
      }
    }
  }

  // 3. Check Accident Conditions
  bool impactDetected = (abs(ax) > CRASH_IMPACT_THRESHOLD || 
                        abs(ay) > CRASH_IMPACT_THRESHOLD || 
                        abs(az) > CRASH_IMPACT_THRESHOLD);

  bool suddenStopDetected = (lastSpeed > 60.0 && gps.speed.kmph() == 0.0 && 
                           (millis() - lastSpeedUpdateTime) < 500);

  bool rolloverDetected = (abs(tiltAngle) > ROLLOVER_ANGLE);

  // 4. Confirm Accident (Multi-Condition Check)
  if ((impactDetected && suddenStopDetected) || 
      (impactDetected && rolloverDetected) || 
      (suddenStopDetected && rolloverDetected)) {
    accidentConfirmed = true;
  }

  // 5. Trigger Emergency Protocol if Accident Confirmed
  if (accidentConfirmed) {
    logCrashData();
    sendEmergencyAlert();
    triggerBuzzer();
    accidentConfirmed = false;  //
  }

  delay(10);  // 
}

// Helper Functions
void logCrashData() {
  String crashLog = "CRASH DETECTED!\n";
  crashLog += "Time: " + String(millis()) + "ms\n";
  crashLog += "Location: Lat=" + String(gps.location.lat(), 6) + 
              ", Lng=" + String(gps.location.lng(), 6) + "\n";
  crashLog += "Impact Force: " + String(mpu.getAccelZ_mss() / 9.81, 2) + "G\n";
  crashLog += "Tilt Angle: " + String(atan2(mpu.getAccelY_mss(), mpu.getAccelZ_mss()) * 180.0 / PI, 2) + "°";
  
  Serial.println(crashLog);  // 
}

void sendEmergencyAlert() {
  String smsMessage = "EMERGENCY! Vehicle crash detected.\n";
  smsMessage += "Location: http://maps.google.com/?q=" + 
                String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  
  gsmAccess.sendSMS("+254710842120", smsMessage);  //
}

void triggerBuzzer() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);//
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);
  }
}
