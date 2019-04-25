#ifdef ESP32

#include "radio.h"
#include "globals.h"
#include <SPI.h>

#include <LoRa.h>

/*
   Initiate the radio module
*/
void initRadio() {

  // reset radio
  pinMode(RST_LoRa, OUTPUT);

  digitalWrite(RST_LoRa, LOW);
  delay(10);
  digitalWrite(RST_LoRa, HIGH);

  SPI.begin (SCK, MISO, MOSI, SS);
  LoRa.setPins (SS, RST_LoRa, DIO0);

  LoRa.setSPIFrequency(10E6);

  if (! LoRa.begin (RF_FREQ * 1E6)) {
    debug ("Starting LoRa failed!");
    while (1);
  }

  debug("LoRa Init OK!");

  LoRa.setSignalBandwidth(500E3);
  // set tx power
  LoRa.setTxPower(20);

  LoRa.setSpreadingFactor(6);

  // crc on
  LoRa.enableCrc();

}

#endif
