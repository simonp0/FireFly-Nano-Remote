#ifdef ESP32

#include "radio.h"
#include "globals.h"
#include <SPI.h>
#include <LoRa.h>


/*
   Initiate the radio module
*/
void initRadio(){
    // reset radio
    pinMode(RST_LoRa, OUTPUT);
    digitalWrite(RST_LoRa, LOW);
    delay(10);
    digitalWrite(RST_LoRa, HIGH);
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST_LoRa, DIO0);
    //LoRa.setSPI(spiInterface);
    //LoRa.setSPIFrequency(1E6);  //Default = 10E6 
    
    //LoRa.setTxPower(TXPOWER);
    LoRa.setTxPower(TXPOWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(SPREADING_FACTOR);

    LoRa.setCodingRate4(CODING_RATE);
    //    LoRa.setPreambleLength(PREAMBLE_LENGTH);
    //    LoRa.setSyncWord(SYNC_WORD);
    LoRa.enableCrc();
    
    if (!LoRa.begin(RF_FREQ * 1E6)){
        debug("Starting LoRa failed !");
        while (1);
    }
    debug("LoRa Init OK!");
    

    LoRa.setSignalBandwidth(BANDWIDTH); // Remote bugs with lower bandwiths (250E3 , 125E3 )
    // Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, and 250E3

}
#endif
