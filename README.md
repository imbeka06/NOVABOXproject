# NOVABOXproject

Key Features Implemented:
Multi-Condition Accident Detection

Impact Force: Checks for >3G acceleration (adjustable).

Sudden Stop: Detects speed drop from >60 km/h to 0 in <0.5 sec.

Rollover: Monitors tilt angle (>90°).

False-Alarm Prevention: Requires at least two conditions to confirm a crash.

Emergency Actions

Logs crash data (time, location, impact force, tilt angle).

Sends SMS alerts via GSM with Google Maps link.

Triggers a buzzer for nearby notification.

Hardware Compatibility

Works with MPU9250 (accelerometer/gyroscope).

Uses NEO-6M GPS for speed/location.

Supports SIM800L GSM for alerts

How to Test & Calibrate:
Adjust Thresholds in the #define section based on real-world testing.

Example: Reduce CRASH_IMPACT_THRESHOLD to 2.5G if too sensitive.

Test False Positives:

Simulate potholes (short spikes <2G) to ensure no false alarms.

GPS/GSM Validation:

Ensure GPS gets a fix outdoors.

Test SMS alerts with a valid SIM card

Wiring Guide:
Component	Arduino Nano ESP32 Pin
MPU9250 SDA	A4
MPU9250 SCL	A5
GPS RX	TX1 (Pin 17)
GPS TX	RX1 (Pin 16)
GSM RX	TX2 (Pin 8)
GSM TX	RX2 (Pin 9)
Buzzer	D5
LAST STEP:
Flash this code to your Arduino Nano ESP32 using VS Code + PlatformIO or Arduino IDE.

Test with simulated crashes (e.g., shake the sensor violently for impact + tilt).

Optimize thresholds for your specific vehicle dynamics.
