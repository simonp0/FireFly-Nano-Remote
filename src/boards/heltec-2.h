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
#define PIN_BUTTON    12
#define PIN_TRIGGER   32
#define ADC_THROTTLE  ADC1_GPIO38_CHANNEL // ADC1_CHANNEL_2

#define PIN_BATTERY   13

#define HAS_BUTTON KEY_BUILTIN // button "PRG" on board

#define PIN_VIBRO     17

#define HAS_DISPLAY  // OLED-Display on board
#define DISPLAY_ROTATION  1
#define DISPLAY_ROTATION_90 0

#define BATTERY_PROBE ADC2_CHANNEL_4 // uses GPIO7
#define BATT_FACTOR 2 // voltage divider 100k/100k on board

// Pins for LORA chip SPI interface, reset line and interrupt lines
// #define RF_RST  23 // RST_LoRa = 14;
// #define RF_DI2  32 // DIO2 = 34
