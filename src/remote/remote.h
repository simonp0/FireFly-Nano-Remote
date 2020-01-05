
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
const float adjVoltage = 4.1/2.8; // Adjustment factor - Heltec : when battery is full, adjVoltage = 4.1 / displayed value
//2.8 868  /   4.3 434
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

bool requestUpdate = false; //when drawSettingsMenu() sets flag requestUpdate=TRUE : Next run of preparePacket() function catches requestUpdate=TRUE flag -> next packet sent to receiver is a SET_STATE = UPDATE request. Possible only under AppState=MENU


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
    { "Info", "Debug", "TestDebug", "Specs", " ", " ", "Settings"},
    { "Remote", "Calibrate", "Pair", "Auto off", "", ""},
    { "Board", "WIFIupdate",  "Motor Min", "Motor Max", "BatteryMin", "BatteryMax", "TODO.test"},
    { "Lights", "Switch ON", "Switch OFF", "Brake Only", "Settings"},
    { "Receiver", "App Mode", "", "", "", "", ""},
    { "A-Cruise", "ON/OFF", "PushSpeed", "PushTime", "Curr.Spike", "CruiseTime", "CurrentLow" }
    // *** LED ROADLIGHTS ***
  };

enum menu_main { MENU_INFO, MENU_REMOTE, MENU_BOARD, MENU_LIGHT, MENU_RECEIVER, MENU_AUTO_CRUISE };
enum menu_info { INFO_DEBUG, INFO_2, INFO_3, INFO_4, INFO_5, INFO_SETTINGS };
enum menu_remote { REMOTE_CALIBRATE, REMOTE_PAIR, REMOTE_SLEEP_TIMER };
enum menu_board { BOARD_UPDATE, BOARD_MENU_MOTOR_MIN, BOARD_MENU_MOTOR_MAX, BOARD_MENU_BATTERY_MIN, BOARD_MENU_BATTERY_MAX, BOARD_MENU_TEST };
enum menu_light { SWITCH_LIGHT_ON, SWITCH_LIGHT_OFF, SWITCH_LIGHT_BRAKES_ONLY, ROADLIGHT_SETTINGS }; // *** LED ROADLIGHTS ***
enum menu_receiver { SUBM_THROTTLE_MODE };
enum menu_auto_cruise { CRUISE_MENU_AUTO_CRUISE, CRUISE_MENU_PUSHING_SPEED , CRUISE_MENU_PUSHING_TIME, CRUISE_MENU_CRUISE_CURRENT_SPIKE, CRUISE_MENU_AUTO_CRUISE_TIME, CRUISE_MENU_CRUISE_CURRENT_LOW };

float currentMenu = 0;
int subMenu = 0;
int subMenuItem = 0;//= 99;

// set idle mode after using menu
bool menuWasUsed = false;
bool quitMainMenu = false;
// const int BATTERY_CELLS = 10;
// const int BATTERY_TYPE = 0;     // 0: LI-ION | 1: LIPO
// const int MOTOR_POLES = 22;
// const int WHEEL_DIAMETER = 90;
// const int WHEEL_PULLEY = 1;
// const int MOTOR_PULLEY = 1;

//BUTTONS 
#include "remotebutton.cpp"
//RemoteButton(int inputPin, int debounce, int DCgap, int holdTime, int longHoldTime, int memoryDelay)
//RemoteButton* triggerButton = new RemoteButton(PIN_TRIGGER, 50, 250, 300, 1000, 200);
//RemoteButton* powerButton = new RemoteButton(PIN_PWRBUTTON, 50, 250, 300, 1000, 200);
RemoteButton triggerButton(PIN_TRIGGER, 40, 250, 300, 1000, 50);
RemoteButton powerButton(PIN_PWRBUTTON, 80, 250, 300, 1000, 50);

// Button constants
const int RELEASED  = 0;
const int CLICK     = 1;
const int DBL_CLICK = 2;
const int HOLD      = 3;
const int LONG_HOLD = 4;

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
void checkBatteryLevel();
void coreTask(void * pvParameters );
int cruiseControl();
void debugButtons();
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

enum paramSelectorList_page_stage{
  ADJUST_PSL_VALUE,
  SAVE_PSL_VALUE,
  CANCEL_PSL_VALUE,
  DISPLAY_PVS_PAGE  
} myPSLpage = ADJUST_PSL_VALUE;


/*
// #void paramValueSelector(uint8_t myGlobalSettingIndex, String paramName, double minValue, double maxValue, double increment, int decimalPlace, String unitStr);
*/

void paramValueSelector(uint8_t myGlobalSettingIndex, String paramName, double minAdjValue, double maxAdjValue, double adjIncrement, int decimalPlace, String unitStr, String label = " ");

double currentParamAdjValue;
double saveParamAdjValue;
bool initFlag_PVS = 1;
int waitTimeMs = 0;

String VescThrottleMode_label[] = {
    "VESC Remote\n(Nunchuk)",
    "PPM",
    "Current+Rev",
    "RPM",
    "Duty",
    "Regen",
    "Handbrake",
    "POS"
};


// *********************************************************************************************************************************************************************
// *********************************************************************************************************************************************************************
// displays a list of parameters and scrolls with throttle input. Trigger click launch paramValueSelector to adjust currently selected parameter
//int paramSelectorIndexArray1[] = { IDX_BATTERY_MIN, IDX_BATTERY_MAX, IDX_LED_BRIGHTNESS_BRAKE, IDX_LED_BRIGHTNESS_OFF, IDX_MOTOR_POLES, IDX_MOTOR_MAX, IDX_MOTOR_MIN };
//String paramSelectorNameArrray[] = { "Batt.Min", "Batt.Max", "LED.Brake", "LED.Off", "Mot.Poles", "Mot.Max", "Mot.Min"};
void paramSelectorList(int *paramSelectorIndexArray);
int paramSelector_selected = 0;
double currentParamSelectorValue;
bool initFlag_PSL = 0;
int waitTimeMs_PSL = 0;
//GlobalSettingsIndex myGlobalSettingIndex;
int myParamSelectorIndexArray1[] ={
    IDX_MIN_HALL,//remote-CONFIGpacket
    IDX_CENTER_HALL,//remote-CONFIGpacket
    IDX_MAX_HALL,//remote-CONFIGpacket
    IDX_BOARD_ID,//remote-CONFIGpacket
    IDX_AUTO_CRUISE_ON,
    IDX_PUSHING_SPEED,
    IDX_PUSHING_TIME,
    IDX_CRUISE_CURRENT_SPIKE,
    IDX_AUTO_CRUISE_TIME,
    IDX_CRUISE_CURRENT_LOW,
    IDX_MAX_PUSHING_SPEED,
    IDX_AUTO_BRAKE_TIME,
    IDX_AUTO_BRAKE_RELEASE,
    IDX_AUTO_BRAKE_ABORT_MAXSPEED,
    IDX_UART_SPEED,
    IDX_uartPullInterval,
    IDX_UART_TIMEOUT,
    IDX_REMOTE_RX_TIMEOUT, //remote
    IDX_REMOTE_RADIOLOOP_DELAY,//remote
    IDX_REMOTE_LOCK_TIMEOUT,//remote
    IDX_REMOTE_SLEEP_TIMEOUT,//remote
    IDX_DISPLAY_BATTERY_MIN,//remote
    IDX_MOTOR_MIN,//remote
    IDX_MOTOR_MAX,//remote
    IDX_BATTERY_MIN,//remote
    IDX_BATTERY_MAX,//remote
    IDX_MAX_SPEED,
    IDX_MAX_RANGE,
    IDX_BATTERY_CELLS,
    IDX_BATTERY_TYPE,
    IDX_MOTOR_POLES,
    IDX_WHEEL_DIAMETER,
    IDX_WHEEL_PULLEY,
    IDX_MOTOR_PULLEY,
    IDX_LED_BRIGHTNESS_FRONT,
    IDX_LED_BRIGHTNESS_BACK,
    IDX_LED_BRIGHTNESS_BRAKE,
    IDX_LED_BRIGHTNESS_OFF,
    IDX_LED_ROADLIGHT_MODE,
    IDX_THROTTLE_MODE,
    
    IDX_ENDOFARRAY
};