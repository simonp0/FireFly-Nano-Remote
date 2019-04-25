// #include <stdint.h>

/*  Hardware related definitions for TTGO V2.1 Board
// ATTENTION: check your board version!
// This settings are for boards labeled v1.6 on pcb, NOT for v1.5 or older
*/

#define LED_ACTIVE_LOW 1  // Onboard LED is active when pin is LOW

// Pin definitions
#define LED           13   // blue LED on board
#define PIN_BUTTON    0   // RX - pushbutton pin
#define PIN_TRIGGER   10
#define PIN_THROTTLE  A5
#define PIN_BATTERY   A7
#define PIN_VIBRO     6

#define HAS_DISPLAY   // OLED-Display on board
#define DISPLAY_ROTATION  3
#define DISPLAY_ROTATION_90 0

// Pins for I2C interface of OLED Display
#define SDA_OLED   4
#define SCL_OLED   15
#define RST_OLED   16

// Pins for LORA chip SPI interface, reset line and interrupt lines
#define RF_SCK  5
#define RF_CS   8
#define RF_RST  4
#define RF_DI0  3
