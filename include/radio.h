#ifndef _RADIO_H
#define _RADIO_H

// #include <LoRa.h>

// Change to allowed frequency
#define RF_FREQ 868.1
//434.1
//868.1

//#define BAND    868100000   //Hz - you can set band here directly,e.g. 868E6,915E6
//#define PABOOST true
#define TXPOWER 18
#define SPREADING_FACTOR 10 // ranges from { 6 - 12 },default 7 see API docs
#define BANDWIDTH 500000   // Hz was 500e3
#define CODING_RATE 7      // correspond to 4/x where x is the value you give to CODING_RATE - 5 default - ranges from { 4 to 8 }
#define PREAMBLE_LENGTH 8  // 8 default  - { 6 to  65535 } - 6 only valid for fixed length packets
#define SYNC_WORD 0x12     // default 0x12 - 0x34

#ifdef ESP32

  void initRadio();

#elif ARDUINO_SAMD_ZERO

  #include <RH_RF69.h>
  void initRadio(RH_RF69 &rdio);

#endif

#endif
