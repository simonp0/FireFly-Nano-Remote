#include <stdint.h>

/*  Hardware related definitions for Heltec V2 Board
// ATTENTION: check your board version!
// This settings are for boards labeled v2 on pcb
// https://www.aliexpress.com/item/868MHz-915MHz-LoRa-ESP32-Oled-Wifi-SX1276-Module-IOT-with-Antenna-For-Arduino-Electronic-diy-kit/32836246848.html
*/
// The most relevant SX1276 connections are:
//
// DIO0 is connected to pin 26 (with the blue LoRa_IRQ label)
// DIO1 is connected to pin 35 (with the blue LoRa_DIO1 label)
// DIO2 is connected to pin 34 (with the blue LoRa_DIO0 label – very confusing)
//

// Pin definitions
#define LED           25   // LED on board
#define PIN_PWRBUTTON 12
#define PIN_TRIGGER   32
#define ADC_THROTTLE  ADC1_GPIO38_CHANNEL // ADC1_CHANNEL_2

#define PIN_BATTERY   13

//#define PIN_LIGHT      //LED lights control PIN

#define HAS_BUTTON (0) // button "PRG" on board

#define PIN_VIBRO     17

#define HAS_DISPLAY  // OLED-Display on board
#define DISPLAY_ROTATION  1
#define DISPLAY_ROTATION_90 0

#define BATTERY_PROBE ADC2_CHANNEL_4 // uses GPIO7
#define BATT_FACTOR 2 // voltage divider 100k/100k on board
#define VEXT 21

// Pins for I2C interface of OLED Display
#define DISPLAY_SDA 4
#define DISPLAY_SCL 15
#define DISPLAY_RST 16

// Pins for LORA chip SPI interface, reset line and interrupt lines
#define RF_SCK  5
#define RF_CS   18
#define RF_MISO 19
#define RF_MOSI 27
#define RF_RST  23
#define RF_DI0  26
#define RF_DI1  33
#define RF_DI2  32
