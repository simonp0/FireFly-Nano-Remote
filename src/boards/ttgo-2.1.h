#include <stdint.h>

/*  Hardware related definitions for TTGO V2.1 Board
// ATTENTION: check your board version!
// This settings are for boards labeled v1.6 on pcb, NOT for v1.5 or older
*/

// Pin definitions
#define LED           25   // LED on board
#define PIN_BUTTON    14   // RX - pushbutton pin
#define PIN_TRIGGER   RX
#define PIN_THROTTLE  34
#define PIN_BATTERY   35

// #define PIN_VIBRO     ?

#define HAS_DISPLAY  // OLED-Display on board
#define DISPLAY_ROTATION  3

#define HAS_BATTERY_PROBE ADC1_GPIO35_CHANNEL // uses GPIO7
#define BATT_FACTOR 2 // voltage divider 100k/100k on board

// Pins for I2C interface of OLED Display
#define SDA_OLED 21
#define SCL_OLED 22
#define RST_OLED 16

// Pins for LORA chip SPI interface, reset line and interrupt lines
// #define SCK  5
#define RF_CS   18
// #define MISO 19
// #define MOSI 27
#define RST_LoRa  23
#define DIO0  26
#define DIO1  33
#define DIO2  32
