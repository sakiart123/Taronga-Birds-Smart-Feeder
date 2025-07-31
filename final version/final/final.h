#ifndef BIRD_FEEDER_SYSTEM_H
#define BIRD_FEEDER_SYSTEM_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// --- Global Constants and Pin Definitions ---
extern const int chipSelect;
extern const char* filename;
extern const char* whiteList;

#define BIRD_LED_PIN 11
#define BATTERY_LED_PIN_R 13
#define BATTERY_LED_PIN_G 12
#define MAX_CMD_LEN 64
#define USERVO_BAUDRATE (uint32_t)115200
#define DEBUG_SERIAL Serial
#define RFID_SERIAL Serial3

// --- Class Declarations ---

// Manages all SD card interactions: reading config files and writing logs.
class SDcard {
public:
    void begin(uint8_t csPin);                                                  // Initializes the SD card module.
    File open(const char* filename, uint8_t mode);                              // A wrapper for the standard SD.open function.
    bool logInfoAndPrint(const DateTime& dt, const char* birdID, const char* filename); // Logs a bird visit with a full timestamp.
    bool logInfoAndPrint(const char* birdID, const char* filename);             // Logs a bird visit without a timestamp.
    bool checkWhitelist(const String& id);                                      // Checks if a given ID exists in WList.txt.
    bool isWithinOperatingHours(const DateTime& curTime);                       // Checks if current time is within the hours defined in config.txt.
    unsigned long getCooldownMillis();                                          // Reads the feeder cooldown duration from config.txt.
    void getPollingRates(unsigned long &onTimeMillis, unsigned long &offTimeMillis); // Reads antenna on/off cycle times from config.txt.
};

// A wrapper for the Real-Time Clock (RTC) module to handle timekeeping.
class rtcClass {
private:
    RTC_DS3231 rtc;
public:
    void rtcInit();                                                             // Initializes the RTC and sets time if power was lost.
    DateTime now();                                                             // Gets the current date and time from the RTC.
    bool rtcTrigger(const DateTime& at, bool daily, void (*cb)());              // Triggers a callback function at a specific time.
};

// Controls a simple single-color LED.
class Led {
private:
    int _pin;
public:
    explicit Led(int pin);                                                      // Constructor to set up the LED pin.
    void on();                                                                  // Turns the LED on.
    void off();                                                                 // Turns the LED off.
};

// Controls a two-pin (e.g., Red/Green) LED to create multiple colors.
class RgbLed {
private:
    int _rPin, _gPin;
public:
    RgbLed(int r, int g);                                                       // Constructor to set up the two LED pins.
    void off();                                                                 // Turns both LED elements off.
    void setGreen();                                                            // Sets the LED color to green.
    void setOrange();                                                           // Sets the LED color to orange (by mixing red and green).
};

// Monitors battery voltage using an INA219 sensor.
class PowerMonitor {
private:
    Adafruit_INA219 _ina219;
    RgbLed& _statusLed;
    unsigned long _checkInterval;
    unsigned long _previousMillis;
public:
    PowerMonitor(RgbLed& led, unsigned long interval);                          // Constructor to link to a status LED and set check frequency.
    void begin();                                                               // Initializes the INA219 sensor.
    void update();                                                              // Periodically reads voltage and updates the status LED.
};

// Manages the Fashion Star UART servo for controlling the feeder door.
class servomotor {
private:
    FSUS_Servo &userservo;
    const FSUS_SERVO_SPEED_T slowVelocity;
    const FSUS_INTERVAL_T t_acc;
    const FSUS_INTERVAL_T t_dec;
    const FSUS_POWER_T power;
    char status_str[9];
    float current_angle;
    float current_angle_accurate;
    uint8_t angle_index;
public:
    servomotor(FSUS_Servo &userservo);                                          // Constructor to link the class to a specific servo object.
    void initialize();                                                          // Sets up, calibrates, and moves the servo to its home position.
    void moveAndTrack(float new_target_angle);                                  // Moves the servo to an absolute angle.
    void moveAndTrackOffset(float new_target_angle);                            // Moves the servo by a relative angle from its current position.
    void SerialProcessing();                                                    // (For debugging) Reports servo status and angle over Serial.
    bool ServoStatus();                                                         // Checks if the servo has finished its movement.
};

// Reads the state of a digital Hall effect sensor to detect a magnet.
class HallEffect {
private:
    int _pin;
public:
    explicit HallEffect(int pin);                                               // Constructor to set up the sensor pin.
    bool isMagnetDetected();                                                    // Returns true if a magnet is currently detected.
};

// --- Global Function Declaration ---
void processSerial(servomotor &motor);                                          // (For debugging) Reads a value from Serial to manually move the motor.

#endif // BIRD_FEEDER_SYSTEM_H