#ifdef ESP32

#include "radio.h"
#include "globals.h"
#include <SPI.h>
#include <LoRa.h>

#define BAND    868100000   //Hz - you can set band here directly,e.g. 868E6,915E6
#define PABOOST true
#define TXPOWER 18
#define SPREADING_FACTOR 7 // ranges from { 6 - 12 },default 7 see API docs
#define BANDWIDTH 125000   // Hz was 500e3
#define CODING_RATE 5      // correspond to 4/x where x is the value you give to CODING_RATE - 5 default - ranges from { 4 to 8 }
#define PREAMBLE_LENGTH 8  // 8 default  - { 6 to  65535 } - 6 only valid for fixed length packets
#define SYNC_WORD 0x34     // default 0x12 - 0x34

/*
   Initiate the radio module
*/
void initRadio()
{
    // reset radio
    pinMode(RST_LoRa, OUTPUT);
    digitalWrite(RST_LoRa, LOW);
    delay(10);
    digitalWrite(RST_LoRa, HIGH);
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST_LoRa, DIO0);
    LoRa.setTxPower(TXPOWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(SPREADING_FACTOR);
    LoRa.setSignalBandwidth(BANDWIDTH);
    LoRa.setCodingRate4(CODING_RATE);
    LoRa.setPreambleLength(PREAMBLE_LENGTH);
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.enableCrc();
    if (!LoRa.begin(RF_FREQ * 1E6))
    {
        debug("Starting LoRa failed !");
        while (1);
    }
    debug("LoRa Init OK!");
}
#endif
