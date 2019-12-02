#ifndef _CONST_H
#define _CONST_H

#include <Arduino.h>
#include <datatypes.h>

// ********** OPTIONAL FEATURES ***********************************************

//#define FAKE_UART                         // Comment out after pairing the remote and connecting VESC
//#define DEBUG                             // Uncomment DEBUG if you need to debug the remote
//const uint32_t boardAddress = 0xA9BF713C;
//#include <analogWrite.h>
#define ROADLIGHT_CONNECTED                 // FRONT LIGHT and BACKLIGHT option. Reconfigure 2 pins on the receiver side for FRONTLIGHT and BACKLIGHT
//#define OUTPUT_PPM_THROTTLE               // RECEIVER outputs a THROTTLE PPM/PWM signal on PIN_PPM_THROTTLE
//#define DISABLE_UART_THROTTLE_OUTPUT      // RECEIVER disables setThrottle() via UART

// ********** * * * * * * * * * ***********************************************

const COMM_PACKET_ID VESC_COMMAND = COMM_GET_VALUES; // VESC
// const COMM_PACKET_ID VESC_COMMAND = COMM_GET_UNITY_VALUES; // Enertion Unity

/* AUTOCRUISE SETTINGS
  Endless ride - when remote is off and speed is over 12 km/h for 3 seconds,
  cruise control will be activated when speed drops below 12 km/h.

  Slide the board backwards while standing on it or foot brake
  to produce a spike in the current and stop the board.
*/
static bool  AUTO_CRUISE_ON = false;     // disabled by default
static float PUSHING_SPEED = 12.0;       // km/h
static float PUSHING_TIME = 3.0;         // seconds
static float CRUISE_CURRENT_SPIKE = 5.0; // Amps

// boad will stop after 30s if current is low
static float AUTO_CRUISE_TIME = 30.0;    // seconds
static float CRUISE_CURRENT_LOW = 5.0;   // Amps

// auto stop if remote is off and speed is over 20 km/h
static float MAX_PUSHING_SPEED = 20.0;   // km/h

// Auto stop (in seconds)
static float AUTO_BRAKE_TIME = 5;    // time to apply the full brakes
static int AUTO_BRAKE_RELEASE = 3;     // time to release brakes after the full stop
static float AUTO_BRAKE_ABORT_MAXSPEED = 3; // speed under which it's safe to abort auto brake procedure

// UART
static int UART_SPEED = 115200;
//const int UART_SPEED = 9600;


static uint16_t uartPullInterval = 150;
static int UART_TIMEOUT = 25; // 10ms for 115200 bauds, 100ms for 9600 bauds
static int REMOTE_RX_TIMEOUT = 25; // ms (was 20)
static int REMOTE_RADIOLOOP_DELAY = 50; //ms sending THROTTLE each xx millisecond to the receiver

static int REMOTE_LOCK_TIMEOUT = 10; // seconds to lock throttle when idle
static int REMOTE_SLEEP_TIMEOUT = 180; // seconds to go to sleep mode

// turn off display if battery < 15%
static int DISPLAY_BATTERY_MIN = 15;

// VESC current, for graphs only
static int MOTOR_MIN = -30;
static int MOTOR_MAX = 30;
static int BATTERY_MIN = -30;
static int BATTERY_MAX = 30;

// default board configuration
static int MAX_SPEED = 30;       // KM/H
static int MAX_RANGE = 30;       // KM
static int BATTERY_CELLS = 10;
static int BATTERY_TYPE = 0;     // 0: LI-ION | 1: LIPO
static int MOTOR_POLES = 28;
static int WHEEL_DIAMETER = 105;
static int WHEEL_PULLEY = 1;
static int MOTOR_PULLEY = 1;

static int LED_BRIGHTNESS_FRONT = 90;
static int LED_BRIGHTNESS_BACK = 90;
static int LED_BRIGHTNESS_BRAKE = 255;
static int LED_BRIGHTNESS_OFF = 0;
static int LED_ROADLIGHT_MODE = 0;

#ifdef ROADLIGHT_CONNECTED  // ********** LED ROADLIGHTS ***********************************************
    enum RoadLightState{
        OFF,
        ON,
        BRAKES_ONLY
    };

//    RoadLightState myRoadLightState; //default value on startupTime

#endif                      // ********** LED ROADLIGHTS ***********************************************

//  ######## Flash Storage structure for saving all parameters - ESP32 ########
//  https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences
/*
struct FlashStorageSettings {
    bool valid;
    short minHallValue = MIN_HALL;
    short centerHallValue = CENTER_HALL;
    short maxHallValue = MAX_HALL;
    uint32_t boardID = 0;

    //OPT_LED_BRIGHTNESS_FRONT,
    //OPT_LED_BRIGHTNESS_BACK,
    //OPT_LED_BRIGHTNESS_BRAKE,
    //OPT_LED_ROADLIGHT_MODE
 

    bool  AUTO_CRUISE_ON = ::AUTO_CRUISE_ON;     // disabled by default
    float PUSHING_SPEED = ::PUSHING_SPEED;       // km/h
    float PUSHING_TIME = ::PUSHING_TIME;         // seconds
    float CRUISE_CURRENT_SPIKE = ::CRUISE_CURRENT_SPIKE; // Amps

    // boad will stop after 30s if current is low
    float AUTO_CRUISE_TIME = 30.0;    // seconds
    float CRUISE_CURRENT_LOW = 5.0;   // Amps

    // auto stop if remote is off and speed is over 20 km/h
    float MAX_PUSHING_SPEED = 20.0;   // km/h

    // VESC current, for graphs only
    int MOTOR_MIN = -30;
    int MOTOR_MAX = 30;
    int BATTERY_MIN = -30;
    int BATTERY_MAX = 30;

    // default board configuration
    int MAX_SPEED = 30;       // KM/H
    int MAX_RANGE = 30;       // KM
    int BATTERY_CELLS = 10;
    int BATTERY_TYPE = 0;     // 0: LI-ION | 1: LIPO
    int MOTOR_POLES = 28;
    int WHEEL_DIAMETER = 105;
    int WHEEL_PULLEY = 1;
    int MOTOR_PULLEY = 1;
};
*/
//  ######## Flash Storage structure for saving all parameters - ESP32 ########


#define VERSION 3

// Remote > receiver
struct RemotePacket {
    uint32_t address;   //LoRa packets : max payload = 255bytes 
    // --------------  // keep 4 byte alignment!
    uint8_t  version;  // 1
    uint8_t  command;	 // Throttle | Light | Settings
    uint8_t  data;     // e.g. throttle value
    uint8_t  counter;
    // --------------

    //***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
    int8_t  optParamCommand;//test transmission with biggerPackets
    int8_t  optParamIndex;
    int16_t optParamValue;

    int16_t f2wi(float f) { return f * 100; } // pack float
    float w2fi(int16_t w) { return float(w) / 100; }; // unpack float

    float unpackOptParamValue() { return w2fi(optParamValue); }
    void packOptParamValue(float f) { optParamValue = f2wi(f); }
    //***********  RemotePacket::opt parameter implementation  ***********

}; //end struct declaration

// RemotePacket.COMMANDS :
enum RemotePacketCommand {
    SET_THROTTLE,
    SET_CRUISE,
    GET_CONFIG,
    SET_STATE,
    SET_LIGHT,
    OPT_PARAM_MODE
};

// RemotePacket.optParamCommand :
enum OptionParamCommand {
    SET_OPT_PARAM_VALUE,
    GET_OPT_PARAM_VALUE
};

//***********  OPT_PARAM - local array to store settings INDEX and VALUE  ***********
const uint8_t optionParamArrayLength = 64;
static float localOptParamValueArray[optionParamArrayLength];


// RemotePacket.optParamIndex :
/*
enum OptionParamIndex {
    OPT_LED_BRIGHTNESS_FRONT,
    OPT_LED_BRIGHTNESS_BACK,
    OPT_LED_BRIGHTNESS_BRAKE,
    OPT_LED_ROADLIGHT_MODE,     //ToDo : move RemotePacketCommand::SET_LIGHT functionalities into OptionParamIndex::LED_ROADLIGHT_MODE
    OPT_PARAM_5,
    OPT_PARAM_6,
    OPT_PARAM_7,
    OPT_PARAM_8
};
*/

// GlobalSettingsIndex to be stored in flash memory via Preferences library
enum GlobalSettingsIndex {
    IDX_MIN_HALL,
    IDX_CENTER_HALL,
    IDX_MAX_HALL,
    IDX_BOARD_ID,
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
    IDX_REMOTE_RX_TIMEOUT,
    IDX_REMOTE_RADIOLOOP_DELAY,
    IDX_REMOTE_LOCK_TIMEOUT,
    IDX_REMOTE_SLEEP_TIMEOUT,
    IDX_DISPLAY_BATTERY_MIN,
    IDX_MOTOR_MIN,
    IDX_MOTOR_MAX,
    IDX_BATTERY_MIN,
    IDX_BATTERY_MAX,
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
    IDX_LED_ROADLIGHT_MODE
};



// state machine
enum AppState {
    IDLE,       // remote is not connected
    NORMAL,
    PUSHING,
    CRUISE,
    ENDLESS,
    CONNECTED,  // riding with connected remote
    CONNECTING,
    MENU,
    STOPPING,   // emergency brake when remote has disconnected
    STOPPED,
    PAIRING,
    UPDATE,     // update over WiFi
    COASTING    // waiting for board to slowdown
}; //end enum declaration

// Receiver > remote  3 bytes
struct ReceiverPacket {
    uint8_t type;
    uint8_t chain;	// CRC from RemotePacket
    uint8_t state;   // Mode: Pairing, BT, ...
    uint8_t r2;

    // int32_t testBigPacket;//test transmission with biggerPackets

}; //end struct declaration

// responses type
const uint8_t ACK_ONLY  = 1;
const uint8_t TELEMETRY = 2;
const uint8_t CONFIG    = 3;
const uint8_t BOARD_ID  = 4;
const uint8_t OPT_PARAM_RESPONSE  = 5;

struct InfoPacket {  //extends ReceiverPacket
    ReceiverPacket header;
    // --------------  // keep 4 byte alignment!
    int32_t id;
    // --------------
    uint8_t r0;
    uint8_t r1;
    uint16_t r2;
    // --------------
    uint16_t r3;
    uint16_t r4;
  // --------------
}; //end struct declaration

const int PACKET_SIZE = sizeof(InfoPacket);
const int CRC_SIZE = 1;

// New VESC values 12 + 3
struct TelemetryPacket{ //extends ReceiverPacket

    ReceiverPacket header;
    // -----------------  // keep 4 byte alignment!
//  uint16_t speed;       // km/h * 100
    int16_t speed;       // km/h * 100
    uint8_t tempMotor;
    uint8_t tempFET;
    // -----------------
    uint16_t voltage;     // volts * 100
    uint16_t distance;    // km * 100 - 10m accuracy
    // -----------------
    int16_t motorCurrent; // motor amps * 100
    int16_t inputCurrent; // battery amps * 100
    // -----------------
    //
    uint16_t f2w(float f) { return f * 100; } // pack float
    float w2f(uint16_t w) { return float(w) / 100; }; // unpack float
    int16_t f2wi(float f) { return f * 100; } // pack float
    float w2fi(int16_t w) { return float(w) / 100; }; // unpack float

    float getSpeed() { return w2fi(speed); }
    void setSpeed(float f) { speed = f2wi(f); }

    float getVoltage() { return w2f(voltage); }
    void setVoltage(float f) { voltage = f2w(f); }

    float getDistance() { return w2f(distance); }
    void setDistance(float f) { distance = f2w(f); }

    float getMotorCurrent() { return w2fi(motorCurrent); }
    void setMotorCurrent(float f) { motorCurrent = f2wi(f); }

    float getInputCurrent() { return w2fi(inputCurrent); }
    void setInputCurrent(float f) { inputCurrent = f2wi(f); }

};//end struct declaration

// board setting
struct ConfigPacket {  //extends ReceiverPacket
    ReceiverPacket header;
    // -------------------  // keep 4 byte alignment!
    uint8_t  maxSpeed;	    // m/s
    uint8_t  maxRange;      // km
    uint8_t  batteryCells;
    uint8_t  batteryType;   // 0: Li-ion | 1: LiPo
    // -------------------
    uint8_t  motorPoles;
    uint8_t  wheelDiameter;
    uint8_t  wheelPulley;
    uint8_t  motorPulley;
    // -------------------
    int16_t r1;  // battery amps * 100
    int16_t r2;
    // -------------------
    // ********** LED ROADLIGHTS ***********
//    uint8_t  roadlightAppMode;
//    uint8_t  frontLightBrightness_val;
//    uint8_t  backLightBrightness_val;
//    uint8_t  anythingElseToAdd;
    // ********** LED ROADLIGHTS ***********
    //
    float getMaxSpeed() { return (maxSpeed) / 100; }
    void setMaxSpeed(float f) { maxSpeed = f * 100; }
};  //end struct declaration

struct OptionParamPacket {  //extends ReceiverPacket
    ReceiverPacket header;
    // --------------  // keep 4 byte alignment!
    //***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
    uint8_t  optParamCommand;//test transmission with biggerPackets
    uint8_t  optParamIndex;
    int16_t optParamValue;
    //------------------
    int16_t xx1;
    int16_t xx2;
    //------------------
    int16_t zz1;
    int16_t zz2;
    //------------------
    int16_t f2wi(float f) { return f * 100; } // pack float
    float w2fi(int16_t w) { return float(w) / 100; }; // unpack float

    float unpackOptParamValue() { return w2fi(optParamValue); }
    void packOptParamValue(float f) { optParamValue = f2wi(f); }
    //***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
}; //end struct declaration

const int default_throttle = 127;

#ifdef DEBUG
  #define debug(x) Serial.println (x)
#else
  #define debug(x)
#endif

#endif
