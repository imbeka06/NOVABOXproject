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

// Thresholds (adjust based on your testing.Sijui unataka gani)
#define CRASH_IMPACT_THRESHOLD 3.0  // 3G force
#define SUDDEN_STOP_THRESHOLD 0.5   // 0.5 sec to stop from speed
#define ROLLOVER_ANGLE 90.0         // 90 degrees tilt

// Objects
MPU9250 mpu;
TinyGPSPlus gps;
GSM gsmAccess;

// Variables
float lastSpeed = 0.0;
unsigned long lastSpeedUpdateTime = 0;
bool accidentConfirmed = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU9250
  if (!mpu.begin()) {
    Serial.println("MPU9250 init failed!");
    while (1);
  }
  
  // Initialize GPS
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize GSM
  gsmAccess.begin(9600, GSM_RST);
  
  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  // 1. Read Sensor Data
  mpu.readSensor();
  float ax = mpu.getAccelX_mss() / 9.81;  // Convert to G-force
  float ay = mpu.getAccelY_mss() / 9.81;
  float az = mpu.getAccelZ_mss() / 9.81;
  float tiltAngle = atan2(ay, az) * 180.0 / PI;  // Calculate tilt angle

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
    accidentConfirmed = false;  // Reset for next event
  }

  delay(10);  // Small delay to stabilize readings
}

// Helper Functions
void logCrashData() {
  String crashLog = "CRASH DETECTED!\n";
  crashLog += "Time: " + String(millis()) + "ms\n";
  crashLog += "Location: Lat=" + String(gps.location.lat(), 6) + 
              ", Lng=" + String(gps.location.lng(), 6) + "\n";
  crashLog += "Impact Force: " + String(mpu.getAccelZ_mss() / 9.81, 2) + "G\n";
  crashLog += "Tilt Angle: " + String(atan2(mpu.getAccelY_mss(), mpu.getAccelZ_mss()) * 180.0 / PI, 2) + "°";
  
  Serial.println(crashLog);  // Replace with SD card logging if needed
}

void sendEmergencyAlert() {
  String smsMessage = "EMERGENCY! Vehicle crash detected.\n";
  smsMessage += "Location: http://maps.google.com/?q=" + 
                String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  
  gsmAccess.sendSMS("+254710842120", smsMessage);  // Replace with emergency number.For now it is your number.
}

void triggerBuzzer() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);// well the time is not fixed.you can adjust to your preferenc
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);
  }
}
