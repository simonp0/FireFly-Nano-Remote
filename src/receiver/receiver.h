#include <Arduino.h>
#include <Smoothed.h>
#include "CPU.h"
#include "globals.h"
#include "radio.h"
#include "utils.h"
#include "VescUart.h"
#include <Preferences.h>

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

//Pairing at at sartup:
unsigned long startupPairingWindowMs = 3000; //startup takes about 1.5sec

VescUart UART;

// Data structures
ReceiverPacket recvPacket;
RemotePacket remPacket;
TelemetryPacket telemetry;
ConfigPacket boardConfig;
InfoPacket boardInfo;
OptionParamPacket optParamPacket;

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

bool speedLimiterState = true;

// Endless ride
unsigned long timeSpeedReached;

const float AUTO_BRAKE_INTERVAL = 0.1;  // increase the brake force every 0.1s

bool connected = false;
uint8_t throttle;
uint8_t lastThrottle;

String wifiStatus;
String updateStatus;

unsigned long lastBrakeTime = 0;

float lastSpeedValue = 0;
float currentSpeedValue = 0;

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

//***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
void setOptParamValue(uint8_t myGlobalSettingIndex, float value); // Set a value of a specific setting in the localOptParamValueArray[] & updates the flash memory.
float getOptParamValue(uint8_t myGlobalSettingIndex); // Get a setting value by index from the localOptParamValueArray[]
void updateOptParamVariables(); // Update all local variables from the localOptParamValueArray[] values
//***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********


//  ######## Settings Flash Storage - ESP32 ########
Preferences receiverPreferences;

float loadFlashSetting(uint8_t myGlobalSettingIndex);// Load a setting (index & value)pair from flash memory and update the localOptParamValueArray[] value. Returns the setting (float) value. 
void saveFlashSetting(uint8_t myGlobalSettingIndex, float value);// Save a setting (index & value)pair into flash memory

void refreshAllSettingsFromFlashData();// SETTINGS INITIALIZATION - copy flash data into local variables & into localOptParamValueArray[] . If nothing saved in flash, GLOBALS.H hardcoded default values are used instead

float Lpos=0;

bool reverseLocked = 0;
bool handbrakeON = 0;

enum VTM_STATE {
    VTM_STATE_STOPPED,
    VTM_STATE_DRIVING,
    VTM_STATE_HANDBRAKE,
    VTM_STATE_REVERSE
};
VTM_STATE vtmState = VTM_STATE_STOPPED;
String str_vtm_state;

/*
double smoothTimestamp = 0;
int arraySmoothValue[] = { default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle };
int mySmoothedThrottle = default_throttle;
int myAverageValue;
int smoothValueOverTime(int valueToAdd);
*/

//float mySmoothedSpeed = 0;
//float smoothValue2(float *smoothArray2, float valueToAdd);
//float throttleSmoothArray[] = { default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle, default_throttle };
//float speedSmoothArray[] = {0,0,0,0,0,0,0,0,0,0};



//void saveFSSettings();
//void loadFSSettings();

//void saveFSSettings(){
//    receiverPreferences.begin("FireFlyNano", false);
//    receiverPreferences.putShort("AUTO_CRUISE_ON",  recFSSettings.AUTO_CRUISE_ON);
//    receiverPreferences.end();
//}
//void loadFSSettings(){
//    receiverPreferences.begin("FireFlyNano", false);
//    recFSSettings.AUTO_CRUISE_ON = receiverPreferences.getShort("AUTO_CRUISE_ON", ::AUTO_CRUISE_ON);
//    recFSSettings.PUSHING_SPEED = receiverPreferences.getShort("PUSHING_SPEED", ::
//    receiverPreferences.end();
//}

//  ######## Settings Flash Storage - ESP32 ########


// ******************************** LED ROADLIGHTS - Receiver *****************************
// we have PIN_FRONTLIGHT attributed on what is PIN_VIBRO on the remote control side
// we have PIN_BACKLIGHT attributed on what is PIN_PWRBUTTON on the remote control side

#ifdef ROADLIGHT_CONNECTED
    RoadLightState myRoadLightState = OFF; //default value on startupTime

    const double led_pwm_frequency = 200;
    const uint8_t led_pwm_channel_frontLight = 0; //GPIO channel to use
    const uint8_t led_pwm_channel_backLight = 1; //GPIO channel to use
    const uint8_t led_pwm_resolution = 8;

    unsigned long lastBrakeLightPulse = 0;
    unsigned long brakeLightPulseInterval = 100; //ms between each brakeLightPulse initiation
    unsigned long brakeLightPulseDuration = 50; //ms TBD

    void switchLightOn();
    void switchLightOff();
    void switchLightBrakesOnly();
    void updateBrakeLight();
    void emitBrakeLightPulse(uint_fast32_t value);
#endif  // ******************************** LED ROADLIGHTS - Receiver *****************************

// ******** PPM THROTTLE OUTPUT ********
#ifdef OUTPUT_PPM_THROTTLE
    const double pwm_throttle_frequency = 50; //50Hz standard RC freq - pulse width 1 to 2 ms with 1.5ms = neutral
    const uint8_t pwm_throttle_channel = 2; //GPIO channel to use
    const uint8_t pwm_throttle_resolution = 16;//16bits -> 0 to 65535
    // 20ms (50Hz) = 65535 ---> 1ms = 3276 & 2ms = 6552
    //double ppm_throttle_1ms_position = map(1, 0, (1000/pwm_throttle_frequency), 0, 65535);  
    // MAP throttleValue(0 to 255) -> 3276 to 6552
    uint_fast32_t pwm_throttle_dutyCycle_value;
    void updatePpmThrottleOutput(int8_t myThrottle);
#endif
void disablePpmThrottleOutput();
// ******** PPM THROTTLE OUTPUT ********
