#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <ctype.h>  
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"
// Pin definitions
const int chipSelect = 53;
// Instantiate RTC
RTC_DS3231 rtc;
// Log file name
const char* filename = "birdlog.txt";
const char* whiteList = "WList.txt";



// ------------------------ SD CARD -----------------------------
class SDcard {
  public:
    void begin(uint8_t csPin = chipSelect) {
      Serial.println("Initializing SD card...");
      if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        while (1);
      }
      return;
    }
    File open(const char* filename, uint8_t mode = FILE_READ) {
      return SD.open(filename, mode);
    }

  bool logInfoAndPrint(
                  const DateTime& dt, 
                  const char* birdID = "N/A",
                  const char* filename = "birdlog.txt")     
  {
    File logFile = open(filename, FILE_WRITE);
    if (!logFile) { Serial.println(F("Failed to open log file.")); return false; }

    // Now 'dt' is the DateTime object passed to the function
    logFile.print(dt.timestamp(DateTime::TIMESTAMP_FULL));
    logFile.print(F(" BirdID: "));
    logFile.println(birdID);
    logFile.close();

    Serial.println(F("Logged time to SD card:"));
    Serial.print(dt.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(" BirdID: "));
    Serial.println(birdID);
    return true;
  }

  bool logInfoAndPrint(
                    const char* birdID = "N/A",
                    const char* filename = "birdlog.txt")     
    {
      File logFile = open(filename, FILE_WRITE);
      if (!logFile) { Serial.println(F("Failed to open log file.")); return false; }

      // Now 'dt' is the DateTime object passed to the function
      logFile.print(F(" BirdID: "));
      logFile.println(birdID);
      logFile.close();

      Serial.println(F("Logged time to SD card:"));
      Serial.print(F(" BirdID: "));
      Serial.println(birdID);
      return true;
    }
    

  bool checkWhitelist(const String& id) {
    File file = open("WList.txt");
    if (!file) {
      Serial.println(F("Failed to open WList.txt"));
      return false;
    }

    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();  
      if (line == id) {
        file.close();
        return true;
      }
    }

    file.close();
    return false;
  }

  bool isWithinOperatingHours(const DateTime& curTime) {
    int startH = -1, startM = -1, startS = -1;
    int closeH = -1, closeM = -1, closeS = -1;

    // 1. Read start and close times from config.txt
    File configFile = open("config.txt");
    if (!configFile) {
      Serial.println(F("Error: config.txt not found. Assuming closed."));
      return false; // Failsafe: if no config, assume it's always closed.
    }

    while (configFile.available()) {
      String line = configFile.readStringUntil('\n');
      
      // Find the start time line
      if (line.startsWith("Start Time:")) {
        int openParen = line.indexOf('(');
        int closeParen = line.indexOf(')');
        if (openParen != -1 && closeParen > openParen) {
          // Extract "HH:MM:SS" and use sscanf to parse it into integers
          sscanf(line.substring(openParen + 1, closeParen).c_str(), "%d:%d:%d", &startH, &startM, &startS);
        }
      }
      // Find the close time line
      else if (line.startsWith("Close Time:")) {
        int openParen = line.indexOf('(');
        int closeParen = line.indexOf(')');
        if (openParen != -1 && closeParen > openParen) {
          sscanf(line.substring(openParen + 1, closeParen).c_str(), "%d:%d:%d", &closeH, &closeM, &closeS);
        }
      }
    }
    configFile.close();

    // 2. Validate that the times were found and are in a correct 24-hour format
    bool isStartValid = (startH >= 0 && startH < 24 && startM >= 0 && startM < 60 && startS >= 0 && startS < 60);
    bool isCloseValid = (closeH >= 0 && closeH < 24 && closeM >= 0 && closeM < 60 && closeS >= 0 && closeS < 60);

    if (!isStartValid || !isCloseValid) {
      Serial.println(F("Error: Invalid or missing Start/Close time in config.txt."));
      return false; // Failsafe if times are malformed
    }
    
    // 3. Convert all times to total seconds since midnight for easy comparison
    long startSeconds = startH * 3600L + startM * 60L + startS;
    long closeSeconds = closeH * 3600L + closeM * 60L + closeS;
    long currentSeconds = curTime.hour() * 3600L + curTime.minute() * 60L + curTime.second();

    bool inRange;
    if (startSeconds < closeSeconds) {
      // Normal Case: Start time is before close time (e.g., 08:00 to 18:00)
      inRange = (currentSeconds >= startSeconds && currentSeconds < closeSeconds);
    } else {
      // Overnight Case: Start time is after close time (e.g., 22:00 to 06:00)
      inRange = (currentSeconds >= startSeconds || currentSeconds < closeSeconds);
    }
    
    return inRange;
  }
  unsigned long getCooldownMillis() {
    // Default cooldown if file/value is not found: 5 minutes (300,000 ms)
    // We use 'UL' (Unsigned Long) to prevent integer overflow.
    unsigned long cooldownMillis = 300000UL; 
    
    File configFile = open("config.txt");
    if (configFile) {
      while (configFile.available()) {
        String line = configFile.readStringUntil('\n');
        if (line.startsWith("Cool down:")) {
          // Get the number of minutes after "Cool down: "
          long minutes = line.substring(11).toInt();
          // Convert minutes to milliseconds and update the return value
          cooldownMillis = minutes * 60 * 1000UL;
          break; // Found it, stop reading
        }
      }
      configFile.close();
    } else {
      // Optional: Let the user know the default is being used.
      Serial.println(F("config.txt not found, using default cooldown."));
    }

    // Announce the cooldown time that will be used
    Serial.print(F("Feeder cooldown set to: "));
    Serial.print(cooldownMillis / 1000); // Print as seconds
    Serial.println(F(" seconds."));
    
    return cooldownMillis;
  }

  void getPollingRates(unsigned long &onTimeMillis, unsigned long &offTimeMillis) {
    // Default values if not found in config.txt (in milliseconds)
    onTimeMillis = 10000UL;  // 10 seconds
    offTimeMillis = 10000UL; // 10 seconds

    File configFile = open("config.txt");
    if (configFile) {
      while (configFile.available()) {
        String line = configFile.readStringUntil('\n');
        line.trim();
        
        if (line.startsWith("Power On:")) {
          // Get the number of seconds and convert to milliseconds
          long seconds = line.substring(9).toInt();
          onTimeMillis = seconds * 1000UL;
        }
        else if (line.startsWith("Power OFF:")) {
          // Get the number of seconds and convert to milliseconds
          long seconds = line.substring(10).toInt();
          offTimeMillis = seconds * 1000UL;
        }
      }
      configFile.close();
    } else {
      Serial.println(F("config.txt not found, using default polling rates."));
    }

    Serial.print(F("Antenna Polling Rate -> ON: "));
    Serial.print(onTimeMillis / 1000);
    Serial.print(F("s, OFF: "));
    Serial.print(offTimeMillis / 1000);
    Serial.println(F("s"));
  }
};


// ----------------------------- RTC CLOCK ------------------------ 
class rtcClass {
  public:
    void rtcInit() {
      Serial.println("Initializing RTC...");
      if (!rtc.begin()) {
        Serial.println("Couldn't find RTC, Aborted.");
        while (1);
      }
      if (rtc.lostPower()) {
        Serial.println("RTC lost power, setting to compile time.");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      }
    }

    DateTime now() {
      return rtc.now();
    }

    // rtcTrigger method (unused in this system, but kept for completeness)
    bool rtcTrigger(const DateTime& at, bool daily, void (*cb)())   
    {
      DateTime now = rtc.now();                       
      static uint32_t lastStamp = 0;                  

      uint32_t key;                                   
      if (daily) {
        DateTime today(now.year(), now.month(), now.day(),
                      at.hour(), at.minute(), at.second());
        key = today.unixtime();
      } else {
        key = at.unixtime();
      }

      uint32_t nowEpoch = now.unixtime();

      if (nowEpoch >= key && key != lastStamp) {
        cb();                                         
        lastStamp = key;                              
        return true;                                  
      }
      return false;                                   
    }
};

// ----------------------- LED -------------------------------
// --- Pin Definitions ---
#define BIRD_LED_PIN 11
#define BATTERY_LED_PIN_R 13
#define BATTERY_LED_PIN_G 12

class Led {
private:
    int _pin;

public:
    explicit Led(int pin) : _pin(pin) {
        pinMode(_pin, OUTPUT);
        off();
    }

    void on() {
        digitalWrite(_pin, HIGH);
    }

    void off() {
        digitalWrite(_pin, LOW);
    }
};

class RgbLed {
private:
    int _rPin, _gPin;

public:
    RgbLed(int r, int g) : _rPin(r), _gPin(g) {
        pinMode(_rPin, OUTPUT);
        pinMode(_gPin, OUTPUT);
        off(); 
    }

    void off() {
        digitalWrite(_rPin, LOW);
        digitalWrite(_gPin, LOW);
    }

    void setGreen() {
        digitalWrite(_rPin, LOW);
        digitalWrite(_gPin, HIGH);
    }

    void setOrange() {
        digitalWrite(_rPin, HIGH);
        digitalWrite(_gPin, HIGH);
    }
};

// --------------- POWER MONITOR ---------------------------
class PowerMonitor {
private:
    Adafruit_INA219 _ina219;
    RgbLed& _statusLed;

    unsigned long _checkInterval;
    unsigned long _previousMillis;

public:
    PowerMonitor(RgbLed& led, unsigned long interval): _statusLed(led), _checkInterval(interval), _previousMillis(0) {}
    
    void begin() {
        if (!_ina219.begin()) {
            Serial.println("Failed to find INA219 chip");
            while (1) { delay(10); } 
        }
        Serial.println("INA219 sensor found.");
    }

    void update() {
        if (millis() - _previousMillis < _checkInterval) {
            return; 
        }
        _previousMillis = millis();

        float busVoltage = _ina219.getBusVoltage_V();
        Serial.print("Bus Voltage:   "); 
        Serial.println(busVoltage);

        // Batter health base on 12V battery
        const float nominal = 12.0;
        float percent = (busVoltage / nominal) * 100.0;

        if (percent < 15.0) {
            _statusLed.setOrange();
            Serial.println("Battery Status: Critical (<15%)");
        } else if (percent < 30.0) {
            _statusLed.setGreen();
            Serial.println("Battery Status: Low (<30%)");
        } else {
            _statusLed.off();
            Serial.println("Battery Status: Good (>=30%)");
        }
        Serial.println();
    }
};



// ---------------------- MOTOR -------------------------
#define MAX_CMD_LEN 64
#define USERVO_BAUDRATE (uint32_t)115200
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE (uint32_t)115200 // Change from 115200 Might cause issue
#define PRINT_INTERVAL_MS 50

FSUS_Protocol protocol(&Serial2, USERVO_BAUDRATE);
FSUS_Servo uservo_1(3, &protocol);

char buffer[MAX_CMD_LEN];
unsigned long last_print_time = 0;

class servomotor{
 public:
  servomotor(FSUS_Servo &userservo): userservo(userservo){};
 void initialize(){
    // DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    Serial.setTimeout(50);
    DEBUG_SERIAL.println("Start servo setup");
    userservo.init();
    userservo.setTorque(true);
    userservo.setRawAngle(0.0);
    userservo.wait();
    userservo.queryAngle();
    userservo.calibration(userservo.curAngle, 0.0, userservo.curAngle + 90.0, 90.0);
    current_angle_accurate = 0;
 }
  void moveAndTrack(float new_target_angle) {
      // if (ServoStatus()){
        userservo.setRawAngleMTurnByVelocity(new_target_angle, slowVelocity, t_acc, t_dec, power);
        current_angle_accurate = new_target_angle;
      // }
      // return ServoStatus();  // if it is 0, blocked the request
  }

  void moveAndTrackOffset(float new_target_angle){
      Serial.println("motor moving");
      // if (ServoStatus()){
        current_angle_accurate += new_target_angle;
        userservo.setRawAngleMTurnByVelocity(current_angle_accurate, slowVelocity, t_acc, t_dec, power);
      // }
      // return ServoStatus();  // if it is 0, blocked the request
  }

  void SerialProcessing(){
    static float angle_history[5] = {0}; // 5 histories for each angle
    uint8_t status_values;

    userservo.queryAngle();
    current_angle = userservo.curAngle;
    DEBUG_SERIAL.print(current_angle, 2);
    DEBUG_SERIAL.print(',');
    angle_history[angle_index] = current_angle;

    bool stable = true;
    for (int j = 0; j < 5; j++) {
        float diff = angle_history[j] - current_angle;
        if (diff < 0) diff = -diff;
        if (diff >= 0.5) {
            stable = false;
            break;
        }
    }

    status_values = userservo.queryStatus();
    if (userservo.isStop() || stable) {
        status_values |= 0b00000001;
    } else {
        status_values &= 0b11111110;
    }
    for (int j = 7; j >= 0; j--) {
        status_str[7 - j] = (status_values & (1 << j)) ? '1' : '0';
    }
    DEBUG_SERIAL.println(status_str[7]);
  }

  bool ServoStatus(){ //1 means available, 0 means operating
    return (status_str[7]== '1');
  }


private:
  FSUS_Servo &userservo;
  const FSUS_SERVO_SPEED_T slowVelocity = 100;
  const FSUS_INTERVAL_T t_acc = 300;
  const FSUS_INTERVAL_T t_dec = 300;
  const FSUS_POWER_T power = 20000;
  float accumulated_target_angle = 0.0;
  char last_status = '0';
  char status_str[9];
  float current_angle; 
  float current_angle_accurate; //avoiding accumulated offset
  uint8_t angle_index = 0;
};


void processSerial(servomotor &motor) {
  if (!Serial.available()) return;

  // read up to newline (but leave room for the terminator)
  int len = Serial.readBytesUntil('\n', buffer, MAX_CMD_LEN - 1);
  buffer[len] = '\0';           // make it a C‑string
  String input = String(buffer);
  input.trim();                 // strip whitespace

  if (input.length() == 0) 
    return;                     // ignore empty lines

  float val = input.toFloat();
  motor.moveAndTrackOffset(val);
}


// ---------------------- HAUL EFFECT ------------------------

class HallEffect {
private:
    int _pin; 

public:

    explicit HallEffect(int pin) : _pin(pin) {
        pinMode(_pin, INPUT_PULLUP);
    }

    bool isMagnetDetected() {
        // digitalRead() returns the current state of the pin (HIGH or LOW).
        int sensorState = digitalRead(_pin);

        // Most digital Hall effect sensors output LOW when a magnet is detected.
        // Therefore, we return true if the state is LOW.
        if (sensorState == LOW) {
            return true;
        } else {
            return false;
        }
    }
};


//---------- LOOP ------------
// --- Global Object Instances ---
Led birdLed(BIRD_LED_PIN);
RgbLed batteryLed(BATTERY_LED_PIN_R, BATTERY_LED_PIN_G);
PowerMonitor powerMonitor(batteryLed, 2000);
rtcClass rtc1;
SDcard SD1;
servomotor motor1(uservo_1);

#define RFID_SERIAL Serial3

HallEffect newHall(7);
unsigned long coolDownTime = 0;

// --- Plling rate variable ---
unsigned long pollingOnMillis = 0;
unsigned long pollingOffMillis = 0;
unsigned long lastPollingStateChangeTime = 0;
bool isAntennaPollingOn = true; // Start with the antenna ON

void setup() {
    Serial.begin(9600);
    RFID_SERIAL.begin(9600);
    // powerMonitor.begin();
    // rtc1.rtcInit();
    SD1.begin(chipSelect);
    motor1.initialize();
    coolDownTime = SD1.getCooldownMillis();


    SD1.getPollingRates(pollingOnMillis, pollingOffMillis);
    lastPollingStateChangeTime = millis();

    RFID_SERIAL.write("SRA\r");
    RFID_SERIAL.write("ST2\r"); //Use FDX-B/HDX
    Serial.println("----------------SYSTEM START-----------------");
    delay(5100);

    while (RFID_SERIAL.available()) {
      RFID_SERIAL.read();
    }
}

String curID = "";
bool skipFirst = false;
String tagID = "";
bool motorResetCommandSent = false;
unsigned long lastActiveTime = 0;
bool isOpen = true;
int turnOn = 0;
int turnOff = 0;

// Constructor: DateTime(year, month, day, hour, minute, second)
DateTime testTime = DateTime(2025, 7, 28, 15, 30, 0);
void loop() {
  
  unsigned long curTime = millis();

  if (newHall.isMagnetDetected() == true) {

      if(motorResetCommandSent == false){
        Serial.println("no magnet. Resetting motor position to 0.");
        motor1.moveAndTrack(00.0); 
        motorResetCommandSent = true;
      }
     
    
  } else {
      // Serial.println("yes magnet.");
      motorResetCommandSent = false;


  }

   if (lastActiveTime != 0 && (curTime - lastActiveTime < coolDownTime)) {
      // ---- We are IN the cooldown period ----
      isOpen = false; 
      if (turnOff == 0){ 
        RFID_SERIAL.write("SRD\r"); 
        Serial.println("Cooldown started. Reader disabled.");
        turnOn = 0; 
        Serial.println(curTime - lastActiveTime);
        isAntennaPollingOn = false; 
        lastPollingStateChangeTime = curTime;
      }
      turnOff++; 
  } else  {
      // ---- We are OUT of the cooldown period ----
      isOpen = true;
      if (turnOn == 0){ 
        RFID_SERIAL.write("SRA\r"); 
        Serial.println("Cooldown finished. Reader enabled.");
        turnOff = 0; 
        curID = "";  
      }
      turnOn++; 
      

      if (isAntennaPollingOn) {
        // Antenna is ON, check if it's time to turn OFF
        if (curTime - lastPollingStateChangeTime >= pollingOnMillis) {
          isAntennaPollingOn = false;
          lastPollingStateChangeTime = curTime;
          RFID_SERIAL.write("SRD\r");
          Serial.println("Polling: Antenna OFF");
        }
      } else {
        // Antenna is OFF, check if it's time to turn ON
        if (curTime - lastPollingStateChangeTime >= pollingOffMillis) {
          isAntennaPollingOn = true;
          lastPollingStateChangeTime = curTime;
          RFID_SERIAL.write("SRA\r");
          Serial.println("Polling: Antenna ON");
        }
      }
  }

    
   if (isOpen && isAntennaPollingOn ) {

    Serial.println("ANTENNA READY!!!!!!!!!!!!!!");
    tagID = RFID_SERIAL.readStringUntil('\r');
    tagID.trim();
    if (tagID.length() > 0) {
      Serial.println(tagID);
      birdLed.on();
    
      if (curID != tagID){
      
        if (SD1.checkWhitelist(tagID)) {
          motor1.moveAndTrackOffset(45.0);
          Serial.println("Bird in whitelist");
          SD1.logInfoAndPrint(tagID.c_str(), "birdlog.txt"); // comment this out if need you want to log time as well
          lastActiveTime = curTime;
          Serial.print("Active time: ");          
          Serial.println(lastActiveTime);
          curID = tagID;
          // Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL)); // Comment these into log time alongside bird ID
          // SD1.logInfoAndPrint(now, tagID.c_str(), "birdlog.txt");
        }
      }
    }
    else {
          birdLed.off();
    }
  } else {
    birdLed.off();
  }


  // powerMonitor.update(); // comment this in to 

}