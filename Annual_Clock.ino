//Standard Libraries
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "AccelStepper.h"
#include "esp_sleep.h"

//Custom Libraries
#include "wifi_credentials.h"

// ---------------- CONSTANTS AND GLOBAL VARIABLES----------------------

// WiFi and NTP settings.  IP addresses were added in case of DNS failure.  These should be updated periodically.
const char* zeroPointDateTime = "2024-12-21T00:00:00"; // Zero-point date/time in ISO format. 
const double intervalHours = 6;  //How often would you like for the hand to update the time? Default of 6 hours will have ~0.25 degree movement per update.
const int rehomeInterval = 10; //After how many position updates (controlled by intervalHours) would you like for it to get new NTP time, re-home the stepper, and move arm to date?


const char* ntpServers[] = {
  "time.google.com",
  "time.nist.gov",
  "time.cloudflare.com",
  "pool.ntp.org",
  "time.windows.com",
  "time.apple.com",
  "ntp.ubuntu.com",
  "time.facebook.com",
  "ntp.metas.ch",
  "time.nuri.net",
  "216.239.35.0",   // Google NTP Server
  "216.239.35.4",   // Google NTP Server
  "129.6.15.28",    // NIST NTP Server (time-a.nist.gov)
  "129.6.15.29",    // NIST NTP Server (time-b.nist.gov)
  "129.6.15.30",    // NIST NTP Server (time-c.nist.gov)
  "192.5.41.40",    // US Naval Observatory (tick.usno.navy.mil)
  "192.5.41.41",    // US Naval Observatory (tock.usno.navy.mil)
  "129.132.2.21",   // IBM Zurich (ntp1.uzh.ch)
  "129.132.2.22",   // IBM Zurich (ntp2.uzh.ch)
  "81.169.199.193", // European NTP Server (ntp1.fau.de)
  "81.169.199.194", // European NTP Server (ntp2.fau.de)
  "81.169.199.195"  // European NTP Server (ntp3.fau.de)
};
const int numNtpServers = sizeof(ntpServers) / sizeof(ntpServers[0]);
const int wifiConnectionDelay = 1000;  // Delay for WiFi connection attempts in milliseconds
const int ntpUpdateDelay = 2000;       // Delay for NTP update attempts in milliseconds

// Motor and Sensor Pins
const int motorPin1 = 13;
const int motorPin2 = 12;
const int motorPin3 = 14;
const int motorPin4 = 27;
const int sensorPin = 34;
const int builtInBlueLedPin = 2;
const int minStepCount = 2; // Minimum number of steps required to move the stepper.  

// Motor settings
const int MotorInterfaceType = 8;
const int stepsPerRevolution = 4098;  //Set your steps per revolution here
const int hoursInYear = 8760;  //This is an average based on 365.25 days a year so that leap year is blended.
const float stepsPerHour = (float)stepsPerRevolution / hoursInYear;
const int homingSpeed = -1000;         // Speed for homing the motor. 
const int maxMotorSpeed = 1000;        // Maximum speed for the motor
const int motorAcceleration = 500;     // Acceleration for the motor

// Timing settings
const int sensorThreshold = 0;  //This will depend on the photo interrupter module used
const uint64_t intervalMS = (uint64_t)(intervalHours * 60 * 60 * 1000); // intervalHours Conversion to milliseconds
const uint64_t intervalUS = intervalMS * 1000; // intervalHours Conversion to microseconds for sleep function

// Realtime Clock Variables
RTC_DATA_ATTR unsigned long previousMillis = 0;  //Used for interval update loop
RTC_DATA_ATTR int updateCount = 0; 
RTC_DATA_ATTR time_t lastUpdateTime = 0; // Store last update time
RTC_DATA_ATTR int lastStepperPosition = 0;  // Store last position of the stepper motor before sleep

// NTP Server settings
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServers[0], 0, 60000);

// Initialize stepper pin sequence and configurations
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

// Function declarations
bool connectToWiFi();
bool updateTime();
void homeMotor();
int calculateStepPosition(const char* homeDate, String targetDate);
void moveToPosition(int targetPosition);
void updatePosition();
void checkSerialCommands();
String getCurrentTime();
void testMoveSteps(int steps);
void testMoveToPosition(int position);
void testGoToDateTime(String date);
void testHome();



// ---------------- SETUP----------------------
void setup() {


  Serial.begin(115200); // Initialize serial communication

  Serial.println("A person with one watch knows what time it is--a person with two watches is never sure.");

  // Set sensor pin mode
  pinMode(sensorPin, INPUT);
  pinMode(builtInBlueLedPin, OUTPUT);
  digitalWrite(builtInBlueLedPin, LOW); // Ensure the built-in blue LED is off

  stepper.setMaxSpeed(maxMotorSpeed);
  stepper.setAcceleration(motorAcceleration);



  // Set the current position to the last saved position before sleep
  stepper.setCurrentPosition(lastStepperPosition);

  // Check the update count to see what actions need to take place.
  if (updateCount == 0 || updateCount == rehomeInterval) {

    Serial.println("--FULL HOMING SEQUENCE--");

    // Attempt to connect to WiFi
    bool wifiConnected = connectToWiFi();
    
    // Attempt to update NTP time if WiFi connected
    bool timeUpdated = wifiConnected ? updateTime() : false;

    if (timeUpdated) {

      homeMotor();
      updatePosition();

    } else {

      Serial.println("Using current position and millis for updates due to WiFi/NTP failure.");
      if (updateCount == 0) {
        // Initialize lastUpdateTime based on the current date/time if it's the initial setup
        stepper.setCurrentPosition(stepsPerRevolution);  //Set to the last step since the motor will move counter-clockwise (count down from the max number of steps)
        struct tm tm;
        strptime(zeroPointDateTime, "%Y-%m-%dT%H:%M:%S", &tm);
        lastUpdateTime = mktime(&tm);
      }

    }

    
    updateCount = (updateCount == rehomeInterval) ? 1 : updateCount + 1;

  } else {

    Serial.println("--UPDATE SEQUENCE--");
    lastUpdateTime += intervalHours * 3600;
    Serial.println("Updated lastUpdateTime with maths: " + epochToISO(lastUpdateTime));
    updatePosition();
    updateCount++;

  }

  lastStepperPosition = stepper.currentPosition();

  Serial.print("Last Stepper Position: ");
  Serial.println(lastStepperPosition);

  // Ensure all outputs are disabled before sleeping
  stepper.disableOutputs();
  digitalWrite(builtInBlueLedPin, LOW); // Turn off the built-in blue LED

  // Power down WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.print("Update Count: ");
  Serial.println(updateCount);

  // Debug print to confirm interval in microseconds
  Serial.print("Entering deep sleep for ");
  Serial.print(intervalUS);
  Serial.print(" microseconds (");
  Serial.print(intervalHours);
  Serial.println(" hours)");

  // Enter deep sleep for the interval duration.  Comment out these lines if you'd like to use the test commands.
  esp_sleep_enable_timer_wakeup(intervalUS);
  esp_deep_sleep_start();
}

// ---------------- LOOP----------------------
void loop() {
  // The loop will be empty because the main work is handled in the setup function before going to deep sleep

  //These commands are for testing functionality.  You need to uncomment the below and then comment out the lines related to the sleep commands in the setup for these to work.

  //checkSerialCommands();

}

// ---------------- CUSTOM FUNCTIONS----------------------

// Function to connect to WiFi
bool connectToWiFi() {
  Serial.println("Attempting to connect to WiFi (" + String(ssid) + ")");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  // Attempt to connect for 30 seconds
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
    delay(wifiConnectionDelay);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("Failed to connect to WiFi (" + String(ssid) + ")");
    return false;
  }
}

// Function to update time
bool updateTime() {
  connectToWiFi();
  delay(ntpUpdateDelay);
  bool timeUpdated = false;

  for (int i = 0; i < numNtpServers; i++) {
    timeClient.end();  // Stop any existing client
    timeClient = NTPClient(ntpUDP, ntpServers[i], 0, 60000);
    timeClient.begin();

    if (timeClient.update()) {
      Serial.print("Time server found (");
      Serial.print(ntpServers[i]);
      Serial.println(")");
      timeUpdated = true;
      break;
    } else {
      Serial.print("Failed to update time with server: ");
      Serial.println(ntpServers[i]);
      delay(ntpUpdateDelay);  // Wait before trying the next server
    }
  }

  if (timeUpdated) {
    lastUpdateTime = timeClient.getEpochTime();
    Serial.println("Updated lastUpdateTime with NTP: " + epochToISO(lastUpdateTime));
    disconnectWiFi();
    return true;
  } else {
    Serial.println("Failed to update time with all servers. Using millis() as fallback.");
    lastUpdateTime = millis() / 1000;  // Use millis as fallback for last update time
    disconnectWiFi();
    return false;
  }
}


// Function to disconnect from WiFi
void disconnectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi disconnected");
}

// Function to convert epoch time to ISO 8601 format
String epochToISO(time_t epochTime) {
  struct tm * timeInfo = localtime(&epochTime);
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", timeInfo);
  return String(buffer);
}

// Function to home the motor
void homeMotor() {
  digitalWrite(builtInBlueLedPin, HIGH);
  Serial.println("Looking for photo interrupter...");
  stepper.enableOutputs();

  // Check if the sensor is initially blocked
  if (analogRead(sensorPin) <= sensorThreshold) {
    Serial.println("Sensor is initially blocked. Moving to unblock...");
    stepper.setSpeed(-homingSpeed); // Move in the opposite direction
    while (analogRead(sensorPin) <= sensorThreshold) {
      stepper.runSpeed();
    }
    Serial.println("Sensor is now unblocked. Ready to detect arm thickness.");
  }

  // Move in the homing direction
  stepper.setSpeed(homingSpeed);

  // Move until the sensor is triggered
  while (analogRead(sensorPin) > sensorThreshold) {
    stepper.runSpeed();
  }

  Serial.println("Found initial trigger point.");

  // Store the initial position
  long initialTriggerPosition = stepper.currentPosition();

  // Continue moving until the sensor is no longer triggered
  while (analogRead(sensorPin) <= sensorThreshold) {
    stepper.runSpeed();
  }

  // Store the end position
  long endTriggerPosition = stepper.currentPosition();

  // Calculate the midpoint
  long armThicknessSteps = endTriggerPosition - initialTriggerPosition;
  long homePosition = initialTriggerPosition + (armThicknessSteps / 2);

  // Move to the midpoint position and set it as the home position
  stepper.moveTo(homePosition);
  stepper.runToPosition();
  stepper.setCurrentPosition(0); // Set the zero point at the true home position

  Serial.print("Arm thickness in steps: ");
  Serial.println(abs(armThicknessSteps));

  digitalWrite(builtInBlueLedPin, LOW);
  stepper.disableOutputs();

  lastStepperPosition = stepper.currentPosition(); // Update lastStepperPosition only after successful homing
}


// Function to calculate the stepper position based on two dates
int calculateStepPosition(const char* homeDate, String targetDate) {
  // Trim any whitespace from the targetDate string
  targetDate.trim();

  // Default to 00:00:00 if time is not provided
  if (targetDate.length() == 10) {
    targetDate += "T00:00:00";
    Serial.println("Time added to targetDate.");
  }

  Serial.print("Home Date: ");
  Serial.println(homeDate);
  Serial.print("Target Date: ");
  Serial.println(targetDate);

  struct tm homeTimeInfo, targetTimeInfo;

  // Parse the date-time strings without considering the year
  sscanf(homeDate, "%*d-%d-%dT%d:%d:%d",
         &homeTimeInfo.tm_mon, &homeTimeInfo.tm_mday,
         &homeTimeInfo.tm_hour, &homeTimeInfo.tm_min, &homeTimeInfo.tm_sec);
  sscanf(targetDate.c_str(), "%*d-%d-%dT%d:%d:%d",
         &targetTimeInfo.tm_mon, &targetTimeInfo.tm_mday,
         &targetTimeInfo.tm_hour, &targetTimeInfo.tm_min, &targetTimeInfo.tm_sec);

  // Set the year to the same value to ignore it in the calculation
  homeTimeInfo.tm_year = 0;
  targetTimeInfo.tm_year = 0;

  // Adjust the month values to the range 0-11
  homeTimeInfo.tm_mon -= 1;
  targetTimeInfo.tm_mon -= 1;

  time_t homeTime = mktime(&homeTimeInfo);
  time_t targetTime = mktime(&targetTimeInfo);

  // Calculate the difference in seconds
  double secondsBetween = difftime(targetTime, homeTime);

  // Handle the case where the difference is negative (target is earlier in the year than home)
  if (secondsBetween < 0) {
    secondsBetween += 365 * 24 * 3600;  // Add one year worth of seconds
  }

  double hoursBetween = secondsBetween / 3600.0;
  int targetPosition = map((int)hoursBetween, 0, hoursInYear, 0, stepsPerRevolution);
  int targetPositionCCW = stepsPerRevolution - targetPosition;

  return targetPositionCCW;
}

// Function to move the stepper to a certain position
void moveToPosition(int targetPosition) {
  int currentPosition = stepper.currentPosition();

  int stepsToMove = abs(targetPosition - currentPosition);

  Serial.print("Current Position: ");
  Serial.println(currentPosition);
  Serial.print("Target Position: ");
  Serial.println(targetPosition);
  Serial.print("Steps to Move: ");
  Serial.println(stepsToMove);

  if (stepsToMove >= minStepCount) {
    Serial.print("Moving to position: ");
    Serial.println(targetPosition);
    stepper.enableOutputs();
    stepper.moveTo(targetPosition);
    stepper.runToPosition();
    stepper.disableOutputs();
    Serial.println("Movement executed.");
  } else {
    Serial.print("Not moving to position: ");
    Serial.println(targetPosition);
    Serial.print("Steps to move (");
    Serial.print(stepsToMove);
    Serial.print(") is less than the minimum step count (");
    Serial.print(minStepCount);
    Serial.println(")");
  }

  lastStepperPosition = stepper.currentPosition(); // Update lastStepperPosition after movement
}

// Function to update the position
void updatePosition() {
  int targetPosition;
  if (WiFi.status() != WL_CONNECTED) {
    targetPosition = calculateStepPosition(zeroPointDateTime, epochToISO(lastUpdateTime));
  } else {
    targetPosition = calculateStepPosition(zeroPointDateTime, getCurrentTime().c_str());
  }
  moveToPosition(targetPosition);
}


// Function to check for serial commands
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("testMoveSteps")) {
      int steps = command.substring(13).toInt();
      testMoveSteps(steps);
    } else if (command.startsWith("testMoveToPosition")) {
      int position = command.substring(19).toInt();
      testMoveToPosition(position);
    } else if (command.startsWith("testGoToDateTime")) {
      String date = command.substring(16);
      testGoToDateTime(date);
    } else if (command == "testHome") {
      testHome();
    }
  }
}

// Function to get the current time as a string
String getCurrentTime() {
  time_t rawTime = lastUpdateTime;
  struct tm * timeInfo = localtime(&rawTime);
  char buffer[25];
  strftime(buffer, 25, "%Y-%m-%dT%H:%M:%S", timeInfo);
  return String(buffer);
}

// Test functions
void testMoveSteps(int steps) {
  stepper.enableOutputs();
  stepper.move(steps);
  stepper.runToPosition();
  stepper.disableOutputs();
}

void testMoveToPosition(int position) {
  moveToPosition(position);
}

void testGoToDateTime(String date) {
  int targetPosition = calculateStepPosition(zeroPointDateTime, date.c_str());
  moveToPosition(targetPosition);
}

void testHome() {
  homeMotor();
}
