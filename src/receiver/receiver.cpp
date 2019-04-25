#include "receiver.h"

#ifdef ARDUINO_SAMD_FEATHER_M0 // Feather M0 w/Radio
  #include <RH_RF69.h>

  // Singleton instance of the radio driver
  RH_RF69 radio(RF_CS, RF_DI0);

#elif ESP32
  #include <LoRa.h>

  // OTA
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>

    // Uart serial
  HardwareSerial MySerial(1);
#endif

#ifdef RECEIVER_SCREEN
  Adafruit_SSD1306 display(DISPLAY_RST);
#endif

Smoothed <double> batterySensor;
Smoothed <double> motorCurrent;

#include "radio.h"

float signalStrength;
float lastRssi;

// Status blink LED
uint8_t statusCode = 0;
bool statusLedState = false;

unsigned long statusCycleTime, previousStatusMillis, currentMillis, startCycleMillis = 0;
unsigned long lastDelay;

// Initiate VescUart class for UART communication

void setup(){ //runs once after powerOn
    delay(1000);    // wait for VESC?
    Serial.begin(115200);//was 9600
    // while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only
    debug("Receiver");
    //loadEEPROMSettings();
    setDefaultEEPROMSettings();
    calculateRatios();
    pinMode(LED, OUTPUT); //LED onBoard


// ******** LED ROADLIGHTS  -   PWM channels initialisation  ********
#ifdef ROADLIGHT_CONNECTED
    ledcSetup(led_pwm_channel_frontLight, led_pwm_frequency, led_pwm_resolution); // configure LED PWM functionalitites
    ledcAttachPin(PIN_FRONTLIGHT, led_pwm_channel_frontLight); // attach the channel to the GPIO to be controlled
    ledcSetup(led_pwm_channel_backLight, led_pwm_frequency, led_pwm_resolution); // configure LED PWM functionalitites
    ledcAttachPin(PIN_BACKLIGHT, led_pwm_channel_backLight); // attach the channel to the GPIO to be controlled
    myRoadLightState = OFF; //default setting when switching on the board
#endif
// ******** LED ROADLIGHTS IMPLEMENTATION ********


    batterySensor.begin(SMOOTHED_EXPONENTIAL, 10);     // 10 seconds average
    motorCurrent.begin(SMOOTHED_AVERAGE, 2);    // 1 sec average
    motorCurrent.add(0);

    UART.setTimeout(UART_TIMEOUT);

    #ifdef ARDUINO_SAMD_FEATHER_M0
        #ifndef FAKE_UART
            UART.setSerialPort(&Serial1);

        Serial1.begin(UART_SPEED);
    #endif

    initRadio(radio);

    #elif ESP32
        #ifndef FAKE_UART
            UART.setSerialPort(&MySerial);
            MySerial.begin(UART_SPEED, SERIAL_8N1, RX, TX);
        #endif

        initRadio();

        xTaskCreatePinnedToCore( //coreTask on core 0
            coreTask,   /* Function to implement the task */
            "coreTask", /* Name of the task */
            10000,      /* Stack size in words */
            NULL,       /* Task input parameter */
            configMAX_PRIORITIES - 1,          /* Priority of the task */
            NULL,       /* Task handle. */
            0);         /* Core where the task should run */
    #endif

    debug("Setup complete - begin listening");

    #ifdef VEXT // power on display
        pinMode(VEXT, OUTPUT);
        digitalWrite(VEXT, LOW);
    #endif

    #ifdef RECEIVER_SCREEN
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
        display.powerOn();
    #endif
}

bool isTelemetryLost() {
    return telemetryTime !=0 // telemetry was active
    && secondsSince(telemetryTime) > 1; // not received recently
}

// safety check
bool isMoving() {

  if (isTelemetryLost()) return true; // assume movement for safety
  return telemetry.getSpeed() != 0; // moving in any direction
}


float batteryPackPercentage( float voltage ) { // Calculate the battery level of the board based on the telemetry voltage

  float maxCellVoltage = 4.2;
  float minCellVoltage;

  if (boardConfig.batteryType == 0) { // Li-ion
    minCellVoltage = 3.1;
  } else { // Li-po
    minCellVoltage = 3.4;
  }

  float percentage = (100 - ( (maxCellVoltage - voltage / boardConfig.batteryCells) / ((maxCellVoltage - minCellVoltage)) ) * 100);

  if (percentage > 100.0) {
    return 100.0;
  } else if (percentage < 0.0) {
    return 0.0;
  }

  return percentage;
}

#ifdef RECEIVER_SCREEN  //Receiver WIFI UPDATE and OLED display functions
    bool prepareUpdate() { //receiver with screen -> WIFI UPDATE mode functions

        // safety checks
        if (isMoving()) return false;
        state = UPDATE;

        // replace this with your WiFi network credentials
        const char* ssid = "sk8"; // e.g. "FBI Surveillance Van #34";
        const char* password = "12345678"; // e.g. "12345678";

        wifiStatus = "Connecting:";
        updateStatus = String(ssid);

        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);

        while (WiFi.waitForConnectResult() != WL_CONNECTED) {
            debug("Connection Failed!");
            delay(3000);
            // ESP.restart();
            return false;
        }

        // Port defaults to 3232
        // ArduinoOTA.setPort(3232);

        // Hostname defaults to esp3232-[MAC]
        // ArduinoOTA.setHostname("myesp32");

        // No authentication by default
        // ArduinoOTA.setPassword("admin");

        // Password can be set with it's md5 value as well
        // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
        // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");


        ArduinoOTA  .onStart([]() {
                        String type;
                        if (ArduinoOTA.getCommand() == U_FLASH)
                        type = "sketch";
                        else // U_SPIFFS
                        type = "filesystem";
                        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                        updateStatus = "Start updating " + type;
                        })
                    .onEnd([]() {
                        Serial.println("\nEnd");
                        })
                    .onProgress([](unsigned int progress, unsigned int total) {
                        updateStatus = "Progress: " + String(progress / (total / 100), 0) + "%";
                        Serial.println(updateStatus);
                        })
                    .onError([](ota_error_t error) {
                        Serial.printf("Error[%u]: ", error);
                        if (error == OTA_AUTH_ERROR) updateStatus = "Auth Failed";
                        else if (error == OTA_BEGIN_ERROR) updateStatus = "Begin Failed";
                        else if (error == OTA_CONNECT_ERROR) updateStatus = "Connect Failed";
                        else if (error == OTA_RECEIVE_ERROR) updateStatus = "Receive Failed";
                        else if (error == OTA_END_ERROR) updateStatus = "End Failed";
                        });

        ArduinoOTA.begin();
        wifiStatus = "IP: " + WiFi.localIP().toString();
        updateStatus = "Waiting...";
    }
    int getStringWidth(String s) { //OLED text display function
        int16_t x1, y1;
        uint16_t w1, h1;
        display.getTextBounds(s, 0, 0, &x1, &y1, &w1, &h1);
        return w1;
    }
    void drawString(String string, int x, int y, const GFXfont *font) {  //OLED text display function
        display.setFont(font);
        display.setCursor(x, y);
        display.print(string);
    }
    void drawStringCentered(String string, int x, int y, const GFXfont *font) {   //OLED text display function
        display.setFont(font);
        x = x - getStringWidth(string) / 2;
        drawString(string, x, y, font);
    }
    String getState() {  //OLED text display function
        switch (state) {
            case IDLE: return "Idle";
            case CONNECTED: return "Normal";  // dB?
            case STOPPING: return "Stopping";
            case STOPPED: return "Stopped";
            case PUSHING: return "Pushing";
            case ENDLESS: return "Cruise";
            case UPDATE: return "Update";
        }
    }
    void updateScreen() { //OLED refresh display function

        display.clearDisplay();

        switch (state) {

            case IDLE:
                drawBattery();
            break;

            case ENDLESS:
                display.setTextColor(WHITE);
                display.setFont(fontDigital);
                display.setCursor(0, 20);
                display.println("CUR: " + String(telemetry.getMotorCurrent(), 1) + " A");
                display.println("SPD: " + String(telemetry.getSpeed(),1));
            break;

            case UPDATE:
                display.setTextColor(WHITE);
                display.setFont(fontDesc);
                display.setCursor(0, 12);
                display.println(wifiStatus);
                display.println(updateStatus);
            break;

            default:
                if (throttle == default_throttle && !isMoving()) {
                    drawBattery();
                }
                else { // riding
                    display.setTextColor(WHITE);
                    display.setFont(fontDigital);
                    display.setCursor(0, 20);
                    display.println("THR: " + String(map(throttle, 0, 255, -100, 100)) + "%");
                    display.println("SPD: " + String(telemetry.getSpeed(),1) + " k");
                }
            break;
        }

        // ---- status ----
        display.setTextColor(WHITE);
        String s = getState() + "  " +
        String(telemetry.getVoltage(), 1) + "v  " +
        String(telemetry.getDistance(), 1) + "km";

        #ifdef FAKE_UART
            s = "Board ID: " + String(boardID, HEX);
        #endif

        drawStringCentered(s, 64, 62, fontDesc);

        display.display();

    }//end updateScreen()
    void drawBattery() {  //OLED battery display function

        display.setTextColor(WHITE);
        if (telemetry.getVoltage() == 0) {
            // no uart connection
            display.setFont();
            display.setCursor(0, 10);
            display.print("No UART data");
// ****************************************LIGHT IMPLEMENTATION*****************************
        //    display.print(" Vp" + String(digitalRead(PIN_FRONTLIGHT)) );
        //    display.print(" Lp" + String(digitalRead(PIN_BACKLIGHT)) );
// ****************************************LIGHT IMPLEMENTATION*****************************

            // remote info
            display.setCursor(0, 25);
            if (!connected)
                display.print("Remote not connected");
            else
                display.print("Signal: " + String(lastRssi, 0) + " dB");
            //    display.setCursor(0, 40);
            //    display.print("Delay: " + String(lastDelay));
        return;
        }

        // --- battery ----
        int w = 120; int h = 46;
        display.drawRect(0, 0, w, h, WHITE);
        display.drawRect(1, 1, w-2, h-2, WHITE);
        display.fillRect(w, 10, 6, h-10-10, WHITE);

        // fill
        float pc = batteryPackPercentage(telemetry.getVoltage());
        int x = w * pc / 100-8;
        display.fillRect(4, 4, x, h-8, WHITE);

        // % value
        if (pc > 50) { // left side
            display.setTextColor(BLACK);
            drawStringCentered(String(pc, 0) + "%", x/2 + 4, 31, fontBig);
        }
        else { // right side
            display.setTextColor(WHITE);
            drawStringCentered(String(pc, 0) + "%", x + (w - x) / 2, 31, fontBig);
        }

    }//end drawBattery()
#endif //END #IFDEF RECEIVER_SCREEN


void loop() { // CORE 1 task launcher - UART data exchange with VESC
              // function LOOP() starts just after setup() & runs continuously !
    // get telemetry;
    getUartData(); // every 250 ms ?
    #ifdef ARDUINO_SAMD_ZERO
        radioExchange();
        stateMachine();
    #elif RECEIVER_SCREEN //refresh receiver OLED screen
        if (state == UPDATE) {ArduinoOTA.handle();
        }
        updateScreen(); // 25 ms
        vTaskDelay(1);
    #endif
}


#ifdef ESP32   // CORE 0 task launcher - radioExchange & stateMachine
    void coreTask( void * pvParameters ){  // core 0
        String taskMessage = "radio task running on core ";
        taskMessage = taskMessage + xPortGetCoreID();
        Serial.println(taskMessage);

        while (true) {  //while loop that runs continuously !
            radioExchange();
            stateMachine();
            vTaskDelay(1);//was 1
        }
    }
#endif //endifdef


void pairingRequest() { // TODO
  // safety checks
  #ifdef FAKE_UART
    setState(PAIRING);
    return;
  #endif
  // todo: confirm pairing
}

bool receiveData(){ // copies buffer data into remPacket
    // copies buffer data into remPacket
    // Reads the received packet in memory buffer, checks if size & CRC is correct
    // Process packet - copy buffer in "remPacket" - check VERSION and boardAddress
    // returns TRUE if valid remPacket has been made available
  uint8_t len = sizeof(RemotePacket) + CRC_SIZE; // 9
  uint8_t buf[len];

  bool received = false;

  #ifdef ARDUINO_SAMD_ZERO

    received = radio.recv(buf, &len);

  #elif ESP32

    int bytes = 0;
    for (int i = 0; i < len; i++) {
      buf[i] = LoRa.read();
      bytes++;
    }
    // lastRssi = LoRa.packetRssi();
    len = bytes;
    received = true;

  #endif

  if (!received) return false;

  // size check
  if (len != sizeof(RemotePacket) + CRC_SIZE) {
    debug("Wrong packet size");
    return false;
  }

  // crc check
  if (CRC8(buf, sizeof(remPacket)) != buf[sizeof(remPacket)]) {
    debug("CRC mismatch");
    return false;
  }

  // process packet
  memcpy(&remPacket, buf, sizeof(remPacket));

  // address check
  #ifdef FAKE_UART
    // accept any address ***********************************************************************************************************
  #else
    if (remPacket.address != boardID) {
      // pairing request?
      if (!isMoving() && remPacket.command == SET_STATE && remPacket.data == PAIRING) {
        // accept any address
      } else {
        Serial.print("Wrong Board ID, please use: 0x");
        Serial.println(String(boardID, HEX));
        return false;
      }
    }
  #endif

  if (remPacket.version != VERSION) {
    Serial.print("Version mismatch!");
  }

  return true;

  // signalStrength = constrain(map(radio.lastRssi(), -77, -35, 0, 100), 0, 100);
}

bool sendPacket(const void * packet, uint8_t len) { //sends any data packet passed in parameter to paired device - returns TRUE if sent

  // calc crc
  uint8_t crc = CRC8(packet, len);

  // struct to buffer
  len += CRC_SIZE;
  uint8_t buf[len];
  memcpy (buf, packet, len);
  buf[len - CRC_SIZE] = crc;

  bool sent = false;

  #ifdef ESP32

    LoRa.beginPacket(len);
    int t = LoRa.write(buf, len);
    LoRa.endPacket();

    sent = t == len;
    // LoRa.receive(sizeof(remPacket) + CRC_SIZE);

  #elif ARDUINO_SAMD_ZERO

    sent = radio.send(buf, len);

    if (sent) radio.waitPacketSent();

  #endif

  return sent;
}

bool sendData(uint8_t response) { //Answers to (response) type by sending the corresponding data packet ACK_ONLY, TELEMETRY, CONFIG, BOARD_ID

  // send packet
  switch (response) {

  case ACK_ONLY: // no extra data to send
    telemetry.header.type = response;
    telemetry.header.chain = remPacket.counter;
    telemetry.header.state = state;
    return sendPacket(&telemetry, sizeof(telemetry));

  case TELEMETRY:
    telemetry.header.type = response;
    telemetry.header.chain = remPacket.counter;
    telemetry.header.state = state;

    if (sendPacket(&telemetry, sizeof(telemetry))) {
      telemetryUpdated = false;
      return true;
    }
    break;

  case CONFIG:
    debug("Sending board configuration");
    boardConfig.header.type = response;
    boardConfig.header.chain = remPacket.counter;

    if (sendPacket(&boardConfig, sizeof(boardConfig))) {
      justStarted = false; // send config once
      return true;
    }
    break;

  case BOARD_ID:
    debug("Sending board ID");
    boardInfo.header.type = response;
    boardInfo.header.chain = remPacket.counter;
    boardInfo.id = boardID;

    if (sendPacket(&boardInfo, sizeof(boardInfo))) {
      return true;
    }
    break;
  }

  return false;
}

bool dataAvailable() { //Returns TRUE when a valid packet has been received in buffer

  #ifdef ARDUINO_SAMD_ZERO

    return radio.available();

  #elif ESP32

    int packetSize = LoRa.parsePacket(sizeof(remPacket) + CRC_SIZE);
    return packetSize > 0;

  #endif
}

void setState(AppState newState) { //called by the stateMachine()

  switch (newState) {

    case IDLE: break;

    case PUSHING:
      timeSpeedReached = millis();
      break;

    case ENDLESS:
      if (isTelemetryLost()) return;
      cruiseControlStart = millis();
      break;

    case CONNECTED:
      switch (state) {
        case UPDATE: return;

        case PUSHING: // monitor data
        case STOPPING:
        case ENDLESS:
        case COASTING:
          if (remPacket.data == default_throttle) return;
          break;
      }
      // prevent auto-stop
      timeoutTimer = millis();
      connected = true;
      break;

    case STOPPING:
    case STOPPED:
      debug("disconnected");
      lastBrakeTime = millis();
      break;

    case UPDATE: break;
    case PAIRING: break;
  }

  // apply state
  state = newState;
}

void radioExchange() {   //receive packet, execute SET_ or GET_ request, send answer
    //radioExchange() is executed by CORE 0 task via coreTask() function
    // controlStatusLed();
    /* Begin listen for transmission */
    if ( dataAvailable() ) {

        // led on
        digitalWrite(LED, HIGH);

        if (receiveData()) {

            receivedData = true;

            // debug( "New package: command " + String(remPacket.command) + ", data " + String(remPacket.data) + ", counter " + String(remPacket.counter) );

            // Send acknowledgement
            uint8_t response = ACK_ONLY;

            switch (remPacket.command) {
                case SET_THROTTLE:
                case SET_CRUISE:
                    setState(CONNECTED); // keep connection
                    if (telemetryUpdated) { response = TELEMETRY; }
                break;

                case SET_STATE:
                    switch (remPacket.data) {
                        case UPDATE:
                            #ifdef ESP32
                            prepareUpdate();
                            #endif
                        break;
                        case PAIRING:
                            pairingRequest();
                            // request confirmed?
                            if (state == PAIRING) response = BOARD_ID;
                        break;
                    }
                break;

                case GET_CONFIG:
                    response = CONFIG;
                break;

            #ifdef ROADLIGHT_CONNECTED  // ************* LED ROADLIGHTS  -  execute command coming from the RemotePacket  *********************
                case SET_LIGHT:
                    // vibrate(2000); TODO : launch via an independant task to avoid introducing any delay here
                    response = ACK_ONLY;
                    // display.clearDisplay();
                    switch (remPacket.data) {
                        case 1:
                            switchLightOn();
                        break;
                        case 0:
                            switchLightOff();
                        break;
                        case 2:  //brake only mode
                            switchLightBrakesOnly();
                        break;
                    }
                break;
            #endif

            } // end switch

            // send config after power on
            if (justStarted) response = CONFIG;

            // // allow remote to receive
            // delay(5);

            if (sendData(response)) {
            // led on
                digitalWrite(LED, LOW);
            // debug("Sent response");
            }

            // control
            switch (remPacket.command) {
                case SET_THROTTLE: // control VESC speed
                // ignore during auto-stop/update/...
                    if (state == CONNECTED) {
                        setThrottle(remPacket.data);
                    }
                break;

                case SET_CRUISE:
                    setCruise(remPacket.data);
                break;
            }

        } else receivedData = false;
    }   // End if( dataAvailable() ) - Stop listening for transmission

} //END radioExchange() function


void autoCruise(uint8_t speed) {

  if (millisSince(lastCruiseControl) > 50) {
    lastCruiseControl = millis();

    setCruise(speed);
  }
}


void stateMachine() { // handle auto-stop, endless mode, etc...

  switch (state) {

    case IDLE: // no remote connected

      setThrottle(default_throttle);
      if (telemetry.getSpeed() >= PUSHING_SPEED) setState(PUSHING);
      break;

    case PUSHING: // pushing with no remote connected

      if (telemetry.getSpeed() < PUSHING_SPEED) { // pushing ended
        if (AUTO_CRUISE_ON) {
          if (secondsSince(timeSpeedReached) > PUSHING_TIME)
            setState(ENDLESS); // start cruise control
          else
            setState(IDLE); // not enough pushing
        }
      } else if (telemetry.getSpeed() > MAX_PUSHING_SPEED) { // downhill
        setState(STOPPING);
      }
      break;

    case ENDLESS: // cruise without remote at ~12 km/h / 7 mph

      autoCruise(PUSHING_SPEED);

      // detect a foot brake /
      if (true) {
        double current = telemetry.getMotorCurrent(); // ~2 amps
        double smoothed = motorCurrent.get();

        // sudden change (> 5 A) after 2 seconds
        if (abs(current - smoothed) > CRUISE_CURRENT_SPIKE && secondsSince(cruiseControlStart) > 2) {
          setState(IDLE);
        }

        // switch to coasting after some time
        if (secondsSince(cruiseControlStart) > AUTO_CRUISE_TIME) {
          // keep cruise control downhill/uphill
          if (abs(current) <= CRUISE_CURRENT_LOW) setState(COASTING);
        }

        motorCurrent.add(current);
      }
      break;

    case COASTING: // waiting for board to slowdown

      setThrottle(default_throttle);
      // avoid ENDLESS > IDLE > PUSHING loop
      if (telemetry.getSpeed() < PUSHING_SPEED) setState(IDLE);
      break;

    case CONNECTED: // remote is connected

      // timeout handling
      if (millisSince(timeoutTimer) > timeoutMax) {
        debug("receiver timeout");

        // No speed is received within the timeout limit.
        connected = false;
        timeoutTimer = millis();

        setState(STOPPING);

        // use last throttle
        throttle = lastThrottle;
      }

      break;

    case STOPPING: // emergency brake when remote has disconnected

      // start braking from zero throttle
      if (throttle > default_throttle) {
        throttle = default_throttle;
      }

      if (secondsSince(lastBrakeTime) > AUTO_BRAKE_INTERVAL) {

        // decrease throttle to brake  127 / 5 * 0.1
        float brakeForce = constrain(default_throttle / AUTO_BRAKE_TIME * AUTO_BRAKE_INTERVAL, 0, 10);

        // apply brakes
        if (throttle > brakeForce) throttle -= brakeForce; else throttle = 0;
        setThrottle(throttle);

        lastBrakeTime = millis();
      }

      // check speed
      if (throttle == 0 && !isMoving()) {
        setState(STOPPED);
      }

      break;

    case STOPPED:

      // release brakes after a few seconds
      if (secondsSince(lastBrakeTime) > AUTO_BRAKE_RELEASE) setState(IDLE);


      break;

    case UPDATE:
      break;

  }
}


void setStatus(uint8_t code){ //TODO

  short cycle = 0;

  // switch(code){
  //   case COMPLETE:  cycle = 500;    break;
  //   case FAILED:    cycle = 1400;   break;
  // }
  //
  // currentMillis = millis();
  //
  // if(currentMillis - startCycleMillis >= statusCycleTime){
  //   statusCode = code;
  //   statusCycleTime = cycle;
  //   startCycleMillis = currentMillis;
  // }
}


void updateSetting( uint8_t setting, uint64_t value){  // Update a single setting value
    // Map remote setting indexes to receiver settings
    switch( setting ){
        case 0: setting = 0; break;  // TriggerMode
        case 7: setting = 1; break;  // ControlMode
        case 11: setting = 2; break; // Address
    }

    setSettingValue( setting, value);
    updateEEPROMSettings();
    // The address has changed, we need to reinitiate the receiver module
    if(setting == 2) {
        // initRadio(radio);
    }
}

/*TODO setCruise
void setCruise ( bool cruise, uint16_t setPoint ){
  if( rxSettings.controlMode == 0 ){

    setThrottle( setPoint );

  }
  else if( rxSettings.controlMode == 1 ){

    setThrottle( setPoint );

  }
  else if( rxSettings.controlMode == 2 ){

    // Setpoint not used (PID by VESC)
    UART.nunchuck.lowerButton = cruise;

    // Make sure the motor doesn't begin to spin wrong way under high load (and don't allow cruise backwards)
    // if( returnData.rpm < 0 ){
    //
    //   UART.nunchuck.lowerButton = false;
    //   UART.nunchuck.valueY = 127;
    //   UART.setNunchuckValues();
    //   UART.setCurrent(0.0);
    //
    // } else{
    //
    //   UART.nunchuck.valueY = 127;
    //   UART.setNunchuckValues();
    //
    // }
  }
} */
uint16_t ackmaniacHysteresis(uint16_t value){
    //remapping function using Ackmaniac's algo

/*
    uint8_t throttleToApply = 127;

        if ( !isTelemetryLost() ) {
        //    if (millisSince(telemetryTime) < 1000) {  //telemetry is fresh -> max reactivity

                float thisBoardMaxSpeed = 30; //boardConfig.getMaxSpeed();
                float mySpeed = 0; // telemetry.getSpeed();

    //            int throttleNewZero = constrain(map(mySpeed, 0, thisBoardMaxSpeed, 128, 255),128,255);
    //            throttleToApply = constrain(map(throttle, 128, 255, throttleNewZero, 255),130,255);
                int throttleNewZero = map(mySpeed, 0, thisBoardMaxSpeed, 128, 255);
                throttleToApply = map(throttle, 128, 255, throttleNewZero, 255);
                //for current mode !

    //            if(mySpeed < 10) throttleToApply = value;
            //    else throttleToApply=170;
            //    telemetry.getInputCurrent()
            //    telemetry.getSpeed()
            //    telemetry.distance
            //    telemetry.inputCurrent //battery
            //    telemetry.motorCurrent //motor
            //    telemetry
            //    }
            //    else { //telemetry is lagging -> slower smoothed adjustments
                //}
        }


*/
    uint16_t ackValue = value; //TODO mapping function
    return ackValue;
}

void setThrottle(uint16_t value){
    // update display
//    throttle = value;
    throttle = ackmaniacHysteresis(value);

    // UART
    #ifndef FAKE_UART
    UART.nunchuck.valueY = value;
    UART.nunchuck.upperButton = false;
    UART.nunchuck.lowerButton = false;
    UART.setNunchuckValues();
    #endif


    // PPM
    //    digitalWrite(throttlePin, HIGH);
    //    delayMicroseconds(map(throttle, 0, 255, 1000, 2000) );
    //    digitalWrite(throttlePin, LOW);


    // remember throttle for smooth auto stop
    lastThrottle = throttle;

    #ifdef ROADLIGHT_CONNECTED
            updateBrakeLight();
    #endif


}

void setCruise(uint8_t speed) {
    // UART
    #ifndef FAKE_UART
    UART.nunchuck.valueY = 127;
    UART.nunchuck.upperButton = false;
    UART.nunchuck.lowerButton = true;
    UART.setNunchuckValues();
    #endif
}

/* void speedControl( uint16_t throttle , bool trigger ){ TODO
   // Kill switch
   if( rxSettings.triggerMode == 0 ){
     if ( trigger == true || throttle < 127 ){
       setThrottle( throttle );
     }
     else{
       setThrottle( default_throttle );
     }
   }

   // Cruise control
   else if( rxSettings.triggerMode == 1 ){
     if( trigger == true ){

       if( cruising == false ){
         cruiseThrottle = throttle;
         cruiseRPM = returnData.rpm;
         cruising = true;
       }

       setCruise( true, cruiseThrottle );

     }else{
       cruising = false;
       setThrottle( throttle );
     }
   }
 }
*/

void calculateRatios() { //   Update values used to calculate speed and distance travelled.
  // Gearing ratio
  gearRatio = (float)boardConfig.motorPulley / (float)boardConfig.wheelPulley;
  // ERPM to Km/h
  ratioRpmSpeed = (gearRatio * 60 * (float)boardConfig.wheelDiameter * 3.14156) / (((float)boardConfig.motorPoles / 2) * 1E6);
  // Pulses to km travelled
  ratioPulseDistance = (gearRatio * (float)boardConfig.wheelDiameter * 3.14156) / (((float)boardConfig.motorPoles * 3) * 1E6);
}

float rpm2speed(long rpm) { // rpm to km/h
  return abs(ratioRpmSpeed * rpm);
}

// rpm to km/h
long speed2rpm(uint8_t speed) {
  return speed / ratioRpmSpeed;
}

// tachometerAbs to km
float tach2dist(long tachometer) {
  return ratioPulseDistance * tachometer;
}

void getUartData(){ //reads VESC data via UART and stores values in telemetry packet
  if (millisSince(lastUartPull) >= uartPullInterval) {

    lastUartPull = millis();

    // debug
    #ifdef FAKE_UART
      batterySensor.add(41 + (rand()%40) / 100.0);
      telemetry.setVoltage(batterySensor.get());
      telemetry.setDistance(rand()%30);
      telemetry.setSpeed(0);
      telemetry.setMotorCurrent(-21);
      telemetry.setInputCurrent(12);
      telemetry.tempFET = 37;
      telemetry.tempMotor = 60;

      telemetryUpdated = true;
      delay(7);
      return;
    #endif

    // Only get what we need
    if ( UART.getVescValues(VESC_COMMAND) ) { //const COMM_PACKET_ID VESC_COMMAND = COMM_GET_VALUES; // VESC command to read buffer's current state and store it into a "dataPackage data" from VescUart.h

      // float dutyCycleNow;
      // float ampHours;
      // float ampHoursCharged;

      // smooth voltage readings
      float voltage = UART.data.inpVoltage;
      batterySensor.add(voltage);

      if (batteryPackPercentage(voltage) > 0) {
        telemetry.setVoltage(batterySensor.get());
      } else { // ESC is off!
        telemetry.setVoltage(voltage);
      }

      telemetry.setSpeed(rpm2speed(UART.data.rpm));
      telemetry.setDistance(tach2dist(UART.data.tachometerAbs));
      telemetry.setMotorCurrent(UART.data.avgMotorCurrent);
      telemetry.setInputCurrent(UART.data.avgInputCurrent);

      // temperature
      telemetry.tempFET = round(UART.data.tempFET);
      telemetry.tempMotor = round(UART.data.tempMotor);
      if (telemetry.tempMotor > 160) telemetry.tempMotor = 0; // no sensor

      // safety check (erroneous value?)
      if (telemetry.getSpeed() > 100) return;

      lastDelay = millis() - lastUartPull;
      telemetryUpdated = true;
      telemetryTime = millis();

    } else {
      // returnData.ampHours       = 0.0;
      // returnData.inpVoltage     = 0.0;
      // returnData.rpm            = 0;
      // returnData.tachometerAbs  = 0;
      debug("No UART dXXa received!");
    }
  }
}

String uint64ToString(uint64_t number){ //STRING to INTEGER conversion
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if(part1 == 0){
    return String(part2, DEC);
  }

  return String(part1, DEC) + String(part2, DEC);
}

String uint64ToAddress(uint64_t number){ //STRING to HEX address conversion
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  return String(part1, HEX) + String(part2, HEX);
}

// SETTINGS functions
void setDefaultEEPROMSettings(){ //Stores board config variables into boardConfig.packet and calls updateEEPROMSettings()

  boardID = CPU::getID();
  Serial.println("Board ID: " + String(boardID, HEX));

  boardConfig.maxSpeed = MAX_SPEED; // 30 km/h
  boardConfig.maxRange = MAX_RANGE;      // km
  boardConfig.batteryCells = BATTERY_CELLS;
  boardConfig.batteryType = BATTERY_TYPE;    // 0: Li-ion | 1: LiPo
  boardConfig.motorPoles = MOTOR_POLES;
  boardConfig.wheelDiameter = WHEEL_DIAMETER;
  boardConfig.wheelPulley = WHEEL_PULLEY;
  boardConfig.motorPulley = MOTOR_PULLEY;

  updateEEPROMSettings();
}

void loadEEPROMSettings(){ //TODO
  bool rewriteSettings = false;

  // Load settings from EEPROM to custom struct
//  EEPROM.get(0, rxSettings);

  // Loop through all settings to check if everything is fine
  // for ( int i = 0; i < numOfSettings; i++ ) {
  //   int val = getSettingValue(i);
  //
  //   // If setting default value is -1, don't check if its valid
  //   if( settingRules[i][0] != -1 )
  //   {
  //     if ( !inRange( val, settingRules[i][1], settingRules[i][2] ) )
  //     {
  //       // Setting is damaged or never written. Rewrite default.
  //       rewriteSettings = true;
  //       setSettingValue(i, settingRules[i][0] );
  //     }
  //   }

  // }

  // if(rxSettings.firmVersion != VERSION){
  //
  //   setDefaultEEPROMSettings();
  //
  // }
  // else if (rewriteSettings == true)
  // {
  //   updateEEPROMSettings();
  // }
  //
  // debug("Settings loaded");
}

void updateEEPROMSettings(){ //TODO    // Write settings to the EEPROM
//  EEPROM.put(0, rxSettings);
}

void setSettingValue(int index, uint64_t value){ //TODO     // Set a value of a specific setting by index.
  // switch (index) {
  //   case 0: rxSettings.triggerMode = value; break;
  //   case 1: rxSettings.controlMode = value; break;
  //   case 2: rxSettings.address = value;     break;
  //
  //   default: /* Do nothing */ break;
  // }
}

int getSettingValue(uint8_t index){//TODO     // Get settings value by index (usefull when iterating through settings).
  // int value;
  // switch (index) {
  //   case 0: value = rxSettings.triggerMode; break;
  //   case 1: value = rxSettings.controlMode; break;
  //
  //   default: return -1;
  // }
  // return value;
}

bool inRange(int val, int minimum, int maximum){ //checks if value is within MIN - MAX range
  return ((minimum <= val) && (val <= maximum));
}



// ****************************************  LED ROADLIGHTS IMPLEMENTATION  *****************************
//void drawLightPage(); // uint8_t lightBrightnessValue


#ifdef ROADLIGHT_CONNECTED  // ************* LED ROADLIGHTS  -  functions  available *********************
    void switchLightOn(){   // executed from RemotePacket command
        ledcWrite(led_pwm_channel_frontLight, dutyCycle_frontLightOn);
        ledcWrite(led_pwm_channel_backLight, dutyCycle_backLightOn);
        myRoadLightState = ON;
    }

    void switchLightOff(){  // executed from RemotePacket command
        ledcWrite(led_pwm_channel_frontLight, dutyCycle_lightOff);
        ledcWrite(led_pwm_channel_backLight, dutyCycle_lightOff);
        myRoadLightState = OFF;
    }

    void switchLightBrakesOnly(){   // executed from RemotePacket command
        ledcWrite(led_pwm_channel_frontLight, dutyCycle_lightOff);
        ledcWrite(led_pwm_channel_backLight, dutyCycle_lightOff);
        myRoadLightState = BRAKES_ONLY;
    }


    void updateBrakeLight(){   // activates BACKLIGHT flashing sequence from setThrottle() function if Throttle is in braking position
        switch(myRoadLightState){
            case OFF:   // do nothing
            break;
            case ON:    // emit a brakeLight flash then go back to BACKLIGHT normal brightness
                emitBrakeLightPulse(dutyCycle_backLightOn);
            break;
            case BRAKES_ONLY:   //emit a brakeLight flash then go back to OFF
                emitBrakeLightPulse(dutyCycle_lightOff);
            break;
        }
    }

    void emitBrakeLightPulse(uint_fast32_t dutyCycle_returnToNormal){    //emits a brakeLight flash and go back to the previous roadlight state
        uint8_t pwm_channel = led_pwm_channel_backLight;
        uint_fast32_t returnDutyCycle = dutyCycle_returnToNormal;
        uint_fast32_t flashDutyCycle = dutyCycle_brakeLight;

            if (millisSince(lastBrakeLightPulse) >= brakeLightPulseInterval) {    // check when was the last brake flash triggered
                if(lastThrottle<(default_throttle*0.75)){   //we don't flash the BRAKELIGHT on very small braking force
                    lastBrakeLightPulse = millis(); //reset coounter
                    ledcWrite(pwm_channel, flashDutyCycle);     //emit a new flash
                }
            }
            else{ //during a brakeLightPulse emission
                if(millisSince(lastBrakeLightPulse) >= brakeLightPulseDuration){ // check if the flash has been ON long enough to be terminated
                            ledcWrite(pwm_channel, returnDutyCycle); // go back to previous state
                }
            }
             //debug unexpected state
            if(lastThrottle >= (default_throttle)){ //debug unexpected state where brakelight is stuck on FLASH brightness after throttle is back in positive throttle values
                lastBrakeLightPulse = millis(); //reset coounter
                ledcWrite(pwm_channel, returnDutyCycle);     //back to previous state
            }
    }

#endif


// **************************************** LED ROADLIGHTS  -  IMPLEMENTATION*****************************
