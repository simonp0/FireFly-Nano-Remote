
#include <Arduino.h>
#include <Smoothed.h>
#include "CPU.h"
#include "globals.h"
#include "radio.h"
#include "utils.h"
#include "VescUart.h"

//#include <analogWrite.h>

#ifdef RECEIVER_SCREEN
  #include <Adafruit_GFX.h>
  #include "Adafruit_SSD1306.h"
  // fonts
  #include "fonts/Lato_Regular_7.h"
  #include "fonts/Digital.h"
  #include "fonts/Pico.h"
  #include <Fonts/Org_01.h> // Adafruit
  #include <Fonts/FreeSans9pt7b.h>
  #include <Fonts/FreeSans12pt7b.h>
#endif

VescUart UART;

// Data structures
ReceiverPacket recvPacket;
RemotePacket remPacket;
TelemetryPacket telemetry;
ConfigPacket boardConfig;
InfoPacket boardInfo;

AppState state = IDLE;

// get MAC address / CPU serial
uint32_t boardID;

// send configuration on power on
bool justStarted = true;

// send only updated telemetry
bool telemetryUpdated = false;
unsigned long telemetryTime = 0;

// Last time data was pulled from VESC
unsigned long lastUartPull = 0;

// Variables to hold values for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

uint32_t timeoutTimer = 0;
bool receivedData = false;
const short timeoutMax = 500;

// Cruise control
uint16_t cruiseThrottle;
uint16_t cruiseRPM;
bool cruising;
unsigned long lastCruiseControl; //
unsigned long cruiseControlStart;

// Endless ride
unsigned long timeSpeedReached;

const float AUTO_BRAKE_INTERVAL = 0.1;  // increase the brake force every 0.1s

bool connected = false;
uint8_t throttle;
uint8_t lastThrottle;

String wifiStatus;
String updateStatus;

unsigned long lastBrakeTime;


#ifdef RECEIVER_SCREEN
const GFXfont* fontDigital = &Segment13pt7b;  // speed, distance, ...
// const GFXfont* fontPico = &Segment6pt7b;      //
const GFXfont* fontDesc = &Dialog_plain_9;    // km/h
const GFXfont* fontMicro = &Org_01;         // connection screen

const GFXfont* fontBig = &FreeSans12pt7b;         // connection screen
const GFXfont* font = &FreeSans9pt7b;         // connection screen

void updateScreen();
void drawBattery();
#endif

bool prepareUpdate();
void acquireSetting();
void calculateRatios();
void controlStatusLed();
void coreTask(void * pvParameters );
bool dataAvailable();
int getSettingValue(uint8_t index);
void stateMachine();
void getUartData();
bool inRange(int val, int minimum, int maximum);
void loadEEPROMSettings();
void radioExchange();
bool receiveData();
bool sendData(uint8_t response);
void sendUartData();
void setDefaultEEPROMSettings();
void setSettingValue(int index, uint64_t value);
void setState(AppState newState);
void setStatus(uint8_t code);
void setThrottle(uint16_t value);
void setCruise(uint8_t speed);
void speedControl(uint16_t throttle , bool trigger );
String uint64ToAddress(uint64_t number);
String uint64ToString(uint64_t number);
void updateEEPROMSettings();
void updateSetting(uint8_t setting, uint64_t value);


// *******************   LED ROADLIGHTS  -  Receiver    *************
    // we have PIN_FRONTLIGHT attributed on the pin used for PIN_VIBRO on the remote control side
    // we have PIN_BACKLIGHT attributed on the pin used for PIN_PWRBUTTON on the remote control side

#ifdef ROADLIGHT_CONNECTED

    enum RoadLightState{    // All possible ROADLIGHTS modes
        OFF,
        ON,
        BRAKES_ONLY,
        DISCO // yes baby !
    };

    RoadLightState myRoadLightState = OFF; //default mode after startup

    const double led_pwm_frequency = 200;   //msec  -  around 30-50 for a perceivable flashy effect - over 100 for perfectly smooth aspect
    const uint8_t led_pwm_channel_frontLight = 0; //GPIO channel to use
    const uint8_t led_pwm_channel_backLight = 1; //GPIO channel to use
    const uint8_t led_pwm_resolution = 8;

    uint_fast32_t dutyCycle_lightOff = 0;
    uint_fast32_t dutyCycle_frontLightOn = 90;   //TODO : value can be changed via the remote menu
    uint_fast32_t dutyCycle_backLightOn = 90;    //TODO : value can be changed via the remote menu
    uint_fast32_t dutyCycle_brakeLight = 255;   //TODO : value can be changed via the remote menu (optional)

    unsigned long lastBrakeLightPulse;
    unsigned long brakeLightPulseInterval = 100; // msec waiting delay between each brakeLightPulse initiation
    unsigned long brakeLightPulseDuration = 50; //  msec

    //uint8_t ROADLIGHT_MODE = 0;
    void switchLightOn();
    void switchLightOff();
    void switchLightBrakesOnly();
    void updateBrakeLight();
    void emitBrakeLightPulse(uint_fast32_t value);
    //void drawLightPage();

#endif  // *******************  LED ROADLIGHTS  -  Receiver    *************
