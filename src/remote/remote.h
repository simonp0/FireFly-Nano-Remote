
#include <Arduino.h>
#include "CPU.h"
#include <globals.h>
#include "utils.h"

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include <Smoothed.h>

#ifdef ARDUINO_SAMD_ZERO

  const int MIN_HALL = 18;
  const int CENTER_HALL = 305;
  const int MAX_HALL = 629;

#elif ESP32

  #include <LoRa.h>
  #include <driver/adc.h>

   // brownout
  #include <driver/rtc_cntl.h>
  #include <driver/rtc_io.h>
  #include <soc/rtc_cntl_reg.h>

  #include <esp_sleep.h>
  #include <esp_deep_sleep.h>

  // flash
  #include <nvs.h>
  #include <nvs_flash.h>
  #include <Preferences.h>

  const int MIN_HALL = 0;
  const int CENTER_HALL = 752;
  const int MAX_HALL = 1023;
  // **************************************** LED ROADLIGHTS *****************************
    int FRONTLIGHT_BRIGHTNESS = 100;  //TEMP : testing Rx settings update from Tx
    int BACKLIGHT_BRIGHTNESS = 100;  //TEMP : testing Rx settings update from Tx
    int BRAKELIGHT_BRIGHTNESS = 255;  //TEMP : testing Rx settings update from Tx

    RoadLightState myRoadLightState = OFF;  //current roadlight mode activated (OFF : default at startup)
    
    bool requestSwitchLight = false; //flag for putting a SET_LIGHT request in the next remotePacket

    //bool requestAdjustLightBrightness = false; //flag for putting a SET_LIGHT_BRIGHTNESS request in the next remotePacket

    enum RoadlightSetting_page_stage{
        ADJUSTING_FRONTLIGHT_BRIGHTNESS,
        ADJUSTING_BACKLIGHT_BRIGHTNESS,
        ADJUSTING_BRAKELIGHT_BRIGHTNESS,
    } myRoadlightSetting_page_stage = ADJUSTING_FRONTLIGHT_BRIGHTNESS; //default mode at startup


  // **************************************** LED ROADLIGHTS *****************************
  
  //***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
  bool requestSendOptParamPacket = false; //flag for putting a OPT_PARAM command in the next remotePacket


  static intr_handle_t s_rtc_isr_handle;

#endif

struct RemoteSettings {
  bool valid;
  short minHallValue = MIN_HALL;
  short centerHallValue = CENTER_HALL;
  short maxHallValue = MAX_HALL;
  uint32_t boardID = 0;
  // **************************************** LED ROADLIGHTS *****************************
  //short frontLightBrightnessValue = FRONTLIGHT_BRIGHTNESS;
  //short backLightBrightnessValue = BACKLIGHT_BRIGHTNESS;
  // **************************************** LED ROADLIGHTS *****************************
} settings;

RemoteSettings tempSettings;

// calibration
enum calibration_stage {
  CALIBRATE_CENTER,
  CALIBRATE_MAX,
  CALIBRATE_MIN,
  CALIBRATE_STOP
} calibrationStage;

// Data structures
ReceiverPacket recvPacket;
RemotePacket remPacket;
TelemetryPacket telemetry;
ConfigPacket boardConfig;
InfoPacket boardInfo;
OptionParamPacket optParamPacket;

bool needConfig = true; // query board confirmation on start

float signalStrength;
float lastRssi;

// Defining struct to hold stats
struct stats {
  float maxSpeed;
  long maxRpm;
  float minVoltage;
  float maxVoltage;
};

enum ui_page {
  PAGE_MAIN,  // speed, battery, distance
  PAGE_EXT,   // current / settings
  PAGE_MENU,
  PAGE_MAX,
  PAGE_DEBUG,
  PAGE_LIGHT_SETTINGS   // **************************************** LED ROADLIGHTS *****************************
} page = PAGE_MAIN;

// Battery monitoring
const float minVoltage = 3.3; // min voltage with vibro motor
const float maxVoltage = 4.1; //Heltec_Lora32_v2 stops charging around here..
const float refVoltage = 3.3; // Feather double-100K resistor divider
const float adjVoltage = 4.1/2.9; // Adjustment factor - Heltec : when battery is full, adjVoltage = 4.1 / displayed value

unsigned long lastBatterySample = 0; // smooth remote voltage



// Hall Effect throttle
uint16_t hallValue;
float throttle;

const uint8_t hallNoiseMargin = 5;
byte hallCenterMargin = 3;

AppState state = CONNECTING;
AppState receiverState;

// OLED display
unsigned long lastSignalBlink;
bool signalBlink = false;
byte counter = 0;
bool displayOn = false;

unsigned long startupTime;

unsigned long lastInteraction; // last time controls were used
unsigned long stopTime;
bool stopped = true;

// Defining variables for radio communication
unsigned long lastTransmission;
short failCount;

unsigned long lastMarker;
unsigned long lastDelay;

// power
bool power = true;
uint8_t shutdownReq = 0;
int batteryLevel = 0; // remote battery

// cruise control
float cruiseSpeed = 0;
int cruiseStartThrottle;
int cruiseThrottle;

bool requestUpdate = false; //when drawSettingsMenu() sets flag requestUpdate=TRUE : Next run of prepatePacket() function catches requestUpdate=TRUE flag -> next packet sent to receiver is a SET_STATE = UPDATE request. Possible only under AppState=MENU


// menu
enum menu_page {
  MENU_MAIN,
  MENU_SUB,
  MENU_ITEM,
} menuPage = MENU_MAIN;



//int thisArraySize =0;
//size_t sizeArray;

const byte subMenus = 7;
const byte mainMenus = 6;

String MENUS[mainMenus][subMenus] = {
    { "Info", "Debug", "TestDebug", "Specs"},
    { "Remote", "Calibrate", "Pair", "Auto off", "", ""},
    { "Board", "WIFIupdate",  "Max Speed", "Range", "Cells", "Battery"},
    { "Lights", "Switch ON", "Switch OFF", "Brake Only", "Settings"},
    { "Receiver", "App Mode", "", "", "", "", ""},
    { "A-Cruise", "ON/OFF", "PushSpeed", "PushTime", "Curr.Spike", "CruiseTime", "CurrentLow" }
    // *** LED ROADLIGHTS ***
  };

enum menu_main { MENU_INFO, MENU_REMOTE, MENU_BOARD, MENU_LIGHT, MENU_RECEIVER, MENU_AUTO_CRUISE };
enum menu_info { INFO_DEBUG };
enum menu_remote { REMOTE_CALIBRATE, REMOTE_PAIR, REMOTE_SLEEP_TIMER };
enum menu_board { BOARD_UPDATE };
enum menu_light { SWITCH_LIGHT_ON, SWITCH_LIGHT_OFF, SWITCH_LIGHT_BRAKES_ONLY, ROADLIGHT_SETTINGS }; // *** LED ROADLIGHTS ***
enum menu_receiver { SUBM_THROTTLE_MODE };
enum menu_auto_cruise { CRUISE_MENU_AUTO_CRUISE, CRUISE_MENU_PUSHING_SPEED , CRUISE_MENU_PUSHING_TIME, CRUISE_MENU_CRUISE_CURRENT_SPIKE, CRUISE_MENU_AUTO_CRUISE_TIME, CRUISE_MENU_CRUISE_CURRENT_LOW };

float currentMenu = 0;
int subMenu = 0;
int subMenuItem = 0;

// set idle mode after using menu
bool menuWasUsed = false;

// const int BATTERY_CELLS = 10;
// const int BATTERY_TYPE = 0;     // 0: LI-ION | 1: LIPO
// const int MOTOR_POLES = 22;
// const int WHEEL_DIAMETER = 90;
// const int WHEEL_PULLEY = 1;
// const int MOTOR_PULLEY = 1;

// Button constants
const int CLICK     = 1;
const int DBL_CLICK = 2;
const int HOLD      = 3;
const int LONG_HOLD = 4;

// Button timing variables
int debounce = 30;          // ms debounce period to prevent flickering when pressing or releasing the button (20ms)
int DCgap = 250;            // max ms between clicks for a double click event
int holdTime = 300;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 1000;    // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

// icons
const unsigned char logo[] PROGMEM = {
 /* 0xff, 0xff, 0xff, 0x80, 0x00, 0x03, 0x80, 0x00, 0x05, 0x80, 0x00, 0x09, 0x80, 0x00, 0x11, 0x80,
	0x00, 0x21, 0x80, 0x00, 0x41, 0x80, 0x00, 0x81, 0x80, 0x01, 0x01, 0x80, 0x02, 0x01, 0x80, 0x04,
	0x01, 0xbf, 0xfb, 0xfd, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01,
	0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80,
	0x00, 0x01, 0x80, 0x00, 0x01, 0xff, 0xff, 0xff*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char transmittingIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char connectedIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char noconnectionIcon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// fonts
#include "fonts/Lato_Regular_7.h"
#include "fonts/Digital.h"
#include "fonts/Pico.h"
#include <fonts/Org_01_5p.h>

const GFXfont* fontDigital = &Segment13pt7b;  // speed, distance, ...
const GFXfont* fontPico = &Segment6pt7b;      //
const GFXfont* fontDesc = &Dialog_plain_9;    // km/h
const GFXfont* fontMicro = &Org_01_5p;         // connection screen

float getBatteryLevel();
float batteryLevelVolts();
float batteryPackPercentage(float voltage );
void calculateThrottle();
int checkButton();
void checkBatteryLevel();
void coreTask(void * pvParameters );
int cruiseControl();
void drawBatteryLevel();
void drawConnectingScreen();
void drawMode();
void drawMainPage();
void drawExtPage();
void drawSettingsMenu();
void drawShutdownScreen();
void drawPairingScreen();
void drawDebugPage();
void drawSignal();
void drawStringCenter(String value, String caption, uint8_t y);
void drawString(String string, int x, int y, const GFXfont *font);
void drawThrottle();
int getStringWidth(String s);
void handleButtons();
void initTransmitter();
void radioLoop();
bool inRange(short val, short minimum, short maximum);
void isr();
bool isShuttingDown();
void loadSettings();
void loop();
bool pressed(int button);
void backToMainMenu();
int readThrottlePosition();
bool receiveData();
bool receivePacket(uint8_t* buf, uint8_t len);
void reset();
bool responseAvailable(uint8_t size);
bool safeCruiseSpeed();
bool sendData();
void setup();
void sleep();
float speed();
bool suddenSpeedChange();
void transmitToReceiver();
bool triggerActive();
bool triggerActiveSafe();
void updateMainDisplay();
void vibrate(int ms);

void vibe(int vibeMode); //vibrations combos

//***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
//const uint8_t optionParamArrayLength = 128;
//float localOptParamValueArray[optionParamArrayLength];

void setOptParamValue(uint8_t myGlobalSettingIndex, float value);
float getOptParamValue(uint8_t myGlobalSettingIndex);

void sendOptParamToReceiver(uint8_t myGlobalSettingIndex);
bool loadOptParamFromReceiver(uint8_t myGlobalSettingIndex);

//bool retrieveAllOptParamFromReceiverAtStartup = true;
//void retrieveAllOptParamFromReceiver();
//***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********


// **************************************** LED ROADLIGHTS *****************************

void drawLightSettingsPage();//test
//void switchLightOn(); //on receiver side only.
//void switchLightOff(); //on receiver side only.
// **************************************** LED ROADLIGHTS *****************************
enum paramValueSelector_page_stage{
  ADJUST_PVS_VALUE,
  SAVE_PVS_VALUE,
  CANCEL_PVS_VALUE
} myPVSpage = ADJUST_PVS_VALUE;
void paramValueSelector(uint8_t myGlobalSettingIndex, String paramName, double minValue, double maxValue, double increment, int decimalPlace, String unitStr);

