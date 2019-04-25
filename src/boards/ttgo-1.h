#include <stdint.h>

/*  Hardware related definitions for TTGO V2.1 Board
// ATTENTION: check your board version!
// This settings are for boards labeled v1.6 on pcb, NOT for v1.5 or older
*/

#define LED_ACTIVE_LOW 1  // Onboard LED is active when pin is LOW
#define HAS_BUTTON (0) // button "PRG" on board

// Pin definitions
#define LED           2   // blue LED on board
#define PIN_BUTTON    3   // RX - pushbutton pin
#define PIN_TRIGGER   17
#define PIN_THROTTLE  34
// #define PIN_VIBRO     ?

#define HAS_DISPLAY  // OLED-Display on board
#define DISPLAY_ROTATION  1

// Pins for I2C interface of OLED Display
#define SDA_OLED 4
#define SCL_OLED 15
#define RST_OLED 16

// Pins for LORA chip SPI interface, reset line and interrupt lines
#define RF_SCK  5
#define RF_CS   18
#define RF_MISO 19
#define RF_MOSI 27
#define RST_LoRa  14
#define DIO0  26
#define DIO1  33
// #define LORA_IO2  LMIC_UNUSED_PIN
