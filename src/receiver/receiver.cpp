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

Smoothed <double> mySmoothedSpeed;
Smoothed <double> mySmoothedThrottle;

#include "radio.h"

float signalStrength;
float lastRssi;

//  Status blink LED
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
    refreshAllSettingsFromFlashData();  //loads the settings from flash memory (if any)
    //loadEEPROMSettings();
    setDefaultEEPROMSettings();//Stores board config variables into boardConfig.packet and calls {not implemented}updateEEPROMSettings()
    calculateRatios();
    pinMode(PIN_LED, OUTPUT); //LED onBoard


    #ifdef ROADLIGHT_CONNECTED  // ******** LED ROADLIGHTS ********
        ledcSetup(led_pwm_channel_frontLight, led_pwm_frequency, led_pwm_resolution); // configure LED PWM functionalitites
        ledcAttachPin(PIN_FRONTLIGHT, led_pwm_channel_frontLight); // attach the channel to the GPIO to be controlled
        ledcSetup(led_pwm_channel_backLight, led_pwm_frequency, led_pwm_resolution); // configure LED PWM functionalitites
        ledcAttachPin(PIN_BACKLIGHT, led_pwm_channel_backLight); // attach the channel to the GPIO to be controlled
    #endif

    #ifdef OUTPUT_PPM_THROTTLE  // ******** PPM THROTTLE OUTPUT ********
        ledcSetup(pwm_throttle_channel, pwm_throttle_frequency, pwm_throttle_resolution); //configure THROTTLE PWM functionalitites
        ledcAttachPin(PIN_PPM_THROTTLE, pwm_throttle_channel); // attach the channel to the GPIO to be controlled
    #endif
    #ifndef OUTPUT_PPM_THROTTLE
        pinMode(PIN_PPM_THROTTLE, OUTPUT);
        digitalWrite(PIN_PPM_THROTTLE, LOW);
    #endif

    batterySensor.begin(SMOOTHED_EXPONENTIAL, 10);     // 10 seconds average
    motorCurrent.begin(SMOOTHED_AVERAGE, 2);    // 1 sec average
    motorCurrent.add(0);

    mySmoothedSpeed.begin(SMOOTHED_AVERAGE, 5);
    mySmoothedThrottle.begin(SMOOTHED_AVERAGE, 10);

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
                display.setFont(fontDesc); //fontDigital
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
                    display.setFont(fontDesc);  //fontDigital
                    display.setCursor(0, 20);
                    display.println("THR: " + String(map(throttle, 0, 255, -100, 100)) + "%" + "   Avg:" + String(mySmoothedThrottle.get(), 0) );
                    display.println("SPD: " + String(telemetry.getSpeed(),1) + " k" + "   Avg:" + String(mySmoothedSpeed.get(), 1) );
                    display.setCursor(0, 45);
                    display.print(">" + String(str_vtm_state) );
                    display.setCursor(90, 45);
                    display.print("SL" + String(speedLimiterState, 10));
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
            // ************ LED ROADLIGHTS *****************************
              display.print(" L:" + String(myRoadLightState) );
              //display.setCursor(0, 40);
              //display.print(" AvgT" + String(mySmoothedThrottle) );
            // ************ LED ROADLIGHTS *****************************

            // remote info
            display.setCursor(0, 25);
            if (!connected)
                display.print("Remote not connected");
            else{
                signalStrength = constrain(map(lastRssi, -100, -50, 0, 100), 0, 100);
                display.print("Signal: " + String(lastRssi, 0) + "dB " + String(signalStrength,0) + "%");
                }
            //    display.setCursor(0, 40);
            //    display.print("Delay: " + String(lastDelay));
            #ifdef DEBUG
                display.setCursor(0, 45);
                if (startupDelay == 0) {startupDelay = millis();} //(int)(millisSince(startupDelay));}
                display.print("Sartup Time: " + String(startupDelay, 10) + "ms");
            #endif
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
            #ifdef ROADLIGHT_CONNECTED
                updateBrakeLight();
            #endif
        }
    }
#endif //endifdef

void pairingRequest() { // TODO
    // safety checks
    if (millis() < startupPairingWindowMs){ //Opens a small window for pairing just after startup
        setState(PAIRING);
        return;
    }
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
        lastRssi = LoRa.packetRssi();
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

        case OPT_PARAM_RESPONSE:
            debug("Sending rx->tx OPT PARAM packet");
            optParamPacket.header.type = response;
            optParamPacket.header.chain = remPacket.counter;

            if (sendPacket(&optParamPacket, sizeof(optParamPacket))) {
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

        case IDLE:
        break;

        case PUSHING:
            timeSpeedReached = millis();
        break;

        case ENDLESS:
            if (isTelemetryLost()) return;
            cruiseControlStart = millis();
        break;

        case CONNECTED:
            switch (state) {
                case UPDATE: 
                    return;
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

        case UPDATE: 
        break;

        case PAIRING:
        break;
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
        digitalWrite(PIN_LED, HIGH);

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

            #ifdef ROADLIGHT_CONNECTED  // ************* LED ROADLIGHTS*********************
                case SET_LIGHT:
                    // vibrate(2000); TODO : launch via an independant task to avoid introducing any delay here
                    response = ACK_ONLY;
                    // display.clearDisplay();
                    switch (remPacket.data) { //remPacket.data = RoadLightState
                        case RoadLightState::ON: //1:
                            switchLightOn();
                        break;
                        case RoadLightState::OFF: //0:
                            switchLightOff();
                        break;
                        case RoadLightState::BRAKES_ONLY: //2:  //brake only mode
                            switchLightBrakesOnly();
                        break;
                    }
                break;
            #endif

                //***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********
                case OPT_PARAM_MODE:
                    //response = CONFIG;
                    switch (remPacket.optParamCommand) {
                        case SET_OPT_PARAM_VALUE:
                            response = ACK_ONLY;
                            setOptParamValue(remPacket.optParamIndex, remPacket.unpackOptParamValue()); 
                            updateOptParamVariables();//reloads new values into local variables
                        break;
                        case GET_OPT_PARAM_VALUE:
                            response = OPT_PARAM_RESPONSE;
                            optParamPacket.optParamCommand = SET_OPT_PARAM_VALUE;
                            optParamPacket.optParamIndex = remPacket.optParamIndex;
                            optParamPacket.packOptParamValue(getOptParamValue(remPacket.optParamIndex));
                        break;
                    } // end switch
                break;
                //***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********

                case SPEED_LIMITER:
                    response = ACK_ONLY;
                    switch (remPacket.data) { //remPacket.data = RoadLightState
                        case 0: 
                            speedLimiterState = false;
                        break;
                        case 1: 
                            speedLimiterState = true;
                        break;
                    }
                break;
            } // end switch

            // send config after power on
            if (justStarted) response = CONFIG;

            // // allow remote to receive
            // delay(5);

            if (sendData(response)) {
            // led on
                digitalWrite(PIN_LED, LOW);
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
            str_vtm_state = "Idle";
            setThrottle(default_throttle);
            if (telemetry.getSpeed() >= PUSHING_SPEED) setState(PUSHING);
        break;

        case PUSHING: // pushing with no remote connected
            str_vtm_state = "Pushing";
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
            str_vtm_state = "Endless";
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
            str_vtm_state = "Coasting";
            setThrottle(default_throttle);
            // avoid ENDLESS > IDLE > PUSHING loop
            if (telemetry.getSpeed() < PUSHING_SPEED) setState(IDLE);
        break;

        case CONNECTED: // remote is connected
            str_vtm_state = "Connected";
            // timeout handling
            if (millisSince(timeoutTimer) > timeoutMax) {
                debug("receiver timeout");
                // No speed is received within the timeout limit.
                connected = false;
                timeoutTimer = millis();
                lastBrakeTime=millis();
                //  Set last speed
                lastSpeedValue = telemetry.getSpeed();
                setState(STOPPING);
                //  Use last throttle in case of reconnection shortly after timeout
                throttle = lastThrottle;
            }
        break;

        case STOPPING: // emergency brake when remote has disconnected 
            str_vtm_state = "Emergency brake";
            currentSpeedValue = telemetry.getSpeed();
            // start braking from zero throttle
            if ((currentSpeedValue > 0) && (throttle > default_throttle)) {  //going forwards
                throttle = default_throttle;
            }
            if ((currentSpeedValue < 0) && (throttle < default_throttle)){  //going backwards
                throttle = default_throttle;
            }

            if (secondsSince(lastBrakeTime) > AUTO_BRAKE_INTERVAL) {
                // decrease throttle to brake  127 / 5 * 0.1
                float brakeForce = constrain(default_throttle / AUTO_BRAKE_TIME * AUTO_BRAKE_INTERVAL, 0, 10);
                // apply brakes
                if (currentSpeedValue > 0) {  //going forwards
                    if (throttle > brakeForce) throttle -= brakeForce; else throttle = 0;
                    setThrottle(throttle);
                }
                if (currentSpeedValue < 0) {  //going backwards
//                    if (throttle > brakeForce) throttle += brakeForce; else throttle = 10;
                    if (throttle < (255-brakeForce)) throttle += brakeForce; else throttle = 255;
                    setThrottle(throttle);
                }
                lastBrakeTime = millis();
            }

            // check speed
            //if (throttle == 0 && !isMoving()) {setState(STOPPED);}
            //if (throttle == 255 && !isMoving()) {setState(STOPPED);}
            if (throttle == 0 && (abs(currentSpeedValue) < AUTO_BRAKE_ABORT_MAXSPEED)) {
                setState(IDLE);
            }
            if (throttle == 255 && (abs(currentSpeedValue) < AUTO_BRAKE_ABORT_MAXSPEED)) {
                setState(IDLE);
            }

            //avoids going backwards after stopping if auto-reverse is enabled within VESC app 
            if (abs(currentSpeedValue) < abs(lastSpeedValue)){
                lastSpeedValue = currentSpeedValue;
            }

            if(abs(currentSpeedValue) > abs(lastSpeedValue*1.5)){   //absolute speed has increased by 50% --> retry braking or abort if speed low enough
                setThrottle(default_throttle); //restart break procedure from zero throttle
                if(abs(currentSpeedValue) < AUTO_BRAKE_ABORT_MAXSPEED){
                    setState(IDLE);   //If speed is low enough -> abort break procedure
                }
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



void setThrottle(uint16_t throttleValue){
    // update display
    throttle = throttleValue;   //update global variable
    // mySmoothedThrottle = smoothValueOverTime(throttleValue);
    double mySpeed = telemetry.getSpeed();

  //  mySmoothedThrottle.add(throttleValue);
    mySmoothedSpeed.add(mySpeed);

    int deadBand = 5;
    float myCurrent;
    float myRpm;
    float myDuty;
    float myHandbrakeCurrent = 5;
    float motor_max_current = MOTOR_MAX;    //max current
    float motor_min_current = MOTOR_MIN;    //max negative current for active braking or cruising backwards
    float motor_max_brake_current = 10;  //max absolute current for regenerative braking
    float regen_brake_min_speed = 1;    // switch to active braking under this speed
    float handbrakeMaxSpeed = 0.2;      // use handbrake under this speed when braking
    float stoppedStateMaxSpeed = 4;     // consider the board stopped up to this speed

    //TODO: test if regen braking value has to be signed and opposite to current direction - UART.setBrakeCurrent()

    // UART
    #ifndef FAKE_UART
        switch(THROTTLE_MODE){
            //0
            case VTM_NUNCHUCK_UART:
    //            inverse_speed_direction = false;
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                if (speedLimiterState == false){
                    UART.nunchuck.valueY = throttleValue;
                    UART.nunchuck.upperButton = false;
                    UART.nunchuck.lowerButton = false;
                    UART.setNunchuckValues();
                }
                else if (speedLimiterState == true){
                    if(abs(mySpeed) <= LIMITED_SPEED_MAX || throttleValue <= 140 ){
                        UART.nunchuck.valueY = throttleValue;
                        UART.nunchuck.upperButton = false;
                        UART.nunchuck.lowerButton = false;
                        UART.setNunchuckValues();
                    }else if (mySpeed > LIMITED_SPEED_MAX && throttleValue > 140){
                        setCruise(LIMITED_SPEED_MAX);
                    }
                }
            break;
            //1
            case VTM_PPM_PIN_OUT:  // ******** PPM THROTTLE OUTPUT ********
 //               inverse_speed_direction = true;
                 //updatePpmThrottleOutput(throttle);
                #ifdef OUTPUT_PPM_THROTTLE
                    if (speedLimiterState == false){
                        mySmoothedThrottle.add(throttleValue);
                        updatePpmThrottleOutput(throttleValue);
                    }else if (speedLimiterState == true){
                        if(throttleValue >= default_throttle){
                            if(abs(mySpeed) <= LIMITED_SPEED_MAX || throttleValue <= 130 ){
                                mySmoothedThrottle.add(throttleValue);
                                updatePpmThrottleOutput(mySmoothedThrottle.get());
                            }else if (mySpeed > LIMITED_SPEED_MAX && throttleValue > 130){
                            // mySmoothedThrottle.add(default_throttle + constrain( ((mySmoothedThrottle.get()-default_throttle) / (4*(mySmoothedSpeed.get()-LIMITED_SPEED_MAX))), 0, 127) );
                                mySmoothedThrottle.add(mySmoothedThrottle.get() - constrain( pow((mySpeed-LIMITED_SPEED_MAX),3), 0, (mySmoothedThrottle.get()-default_throttle)) );
                                //mySmoothedThrottle.add(mySmoothedThrottle.get()*0.95);
                                updatePpmThrottleOutput(mySmoothedThrottle.get());                            
                            }
                        }else if (throttleValue < default_throttle){//quick reaction for breaking
                                updatePpmThrottleOutput(throttleValue);
                        }
                    }

                #endif
            break;

    //BELOW MODES ARE FOR TEST PURPOSE            
            //2
            case VTM_CURRENT_UART:  //current with regen brake & handbrake when stopped
 //               inverse_speed_direction = false;
                //myCurrent = map(throttle, 0, 255, -abs(motor_max_brake_current), motor_max_current);
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                switch (state){
                    case CONNECTED:
                        switch (vtmState){

                            case VTM_STATE_STOPPED:
                                str_vtm_state = "Stopped";
                                //Idle
                                if (throttleValue > (default_throttle - deadBand) && throttleValue < (default_throttle + deadBand)){ //IDLE - remote centered
                                    myCurrent = 0;
                                    UART.setCurrent(myCurrent);
                                } 

                                //Start going forward from stop                        
                                if ((mySpeed >= (-stoppedStateMaxSpeed)) && (throttleValue > (default_throttle + deadBand) )) {  //moving forwards, cruising
                                    myCurrent = map(throttleValue, default_throttle, 255, 0, motor_max_current);
                                    UART.setCurrent(myCurrent);
                                    vtmState = VTM_STATE_DRIVING;               
                                }
                                // Start rolling forward without throttle input
                                if (mySpeed > stoppedStateMaxSpeed ){ vtmState = VTM_STATE_DRIVING;}

                                //Start going backwards from stop
                                if ( (mySpeed <= stoppedStateMaxSpeed) && (throttleValue < (default_throttle - deadBand)) ){   //moving backwards
                                    myCurrent = map (throttleValue, 0, default_throttle, motor_min_current, 0);
                                    UART.setCurrent(myCurrent);
                                    vtmState = VTM_STATE_REVERSE;  
                                }
                                // Start rolling backward without throttle input
                                if (mySpeed < (-stoppedStateMaxSpeed) ){ vtmState = VTM_STATE_REVERSE;}

                            break;

                            case VTM_STATE_DRIVING:
                                
                                //Keep driving, update motor throttle.
                                if ( (mySpeed >= (-handbrakeMaxSpeed)) && (throttleValue > default_throttle) ) {  //moving forwards, cruising
                                    myCurrent = map(throttleValue, default_throttle, 255, 0, motor_max_current);
                                    UART.setCurrent(myCurrent);
                                    str_vtm_state = "Driving";
                                }                                
                                //Braking while moving forwards
                                if ((mySpeed >= regen_brake_min_speed) && (throttleValue < default_throttle)) {  //moving forwards, regen braking
                                    myCurrent = map(throttleValue, 0, default_throttle, -abs(motor_max_brake_current), 0);
                                    UART.setBrakeCurrent(myCurrent);
                                    str_vtm_state = "Drv: regen. braking";
                                } else if ((mySpeed > 0) && (mySpeed < regen_brake_min_speed) && (throttleValue < default_throttle)) {  //moving forwards, slow, active braking
                                    myCurrent = map(throttleValue, 0, default_throttle, motor_min_current, 0); 
                                    str_vtm_state = "Drv: active braking";                                 
                                    //UART.setCurrent(myCurrent); //+ pow(mySpeed+1,2));   // 
                                } 
                                //We just stopped -> activate Handbrake
                                if ( (abs(mySpeed) < handbrakeMaxSpeed) && (throttleValue < default_throttle) ){ 
                                    myCurrent = myHandbrakeCurrent;
                                    UART.setHandbrake(myCurrent);
                                    vtmState = VTM_STATE_HANDBRAKE;
                                }
                                //Exit
                                if (mySpeed < (-handbrakeMaxSpeed)) {
                                    vtmState = VTM_STATE_STOPPED;
                                }
                            break;

                            case VTM_STATE_HANDBRAKE:
                                str_vtm_state = "Handbrake";
                                //keepalive handbrake if not moving
                                if ( (abs(mySpeed) < handbrakeMaxSpeed) && (throttleValue < default_throttle) ){
                                    myCurrent = myHandbrakeCurrent;
                                    UART.setHandbrake(myCurrent);
                                }
                                //too steep, handbrake slips -> active braking
                                if ( (abs(mySpeed) >= handbrakeMaxSpeed) && (throttleValue < default_throttle) ){
                                    myCurrent = mySpeed/abs(mySpeed) * (myHandbrakeCurrent/2 + pow(mySpeed+1,2));
                                    UART.setCurrent(myCurrent);
                                }
                                // Release handbrake when throttle goes back to neutral
                                if (throttleValue >= default_throttle){
                                    vtmState = VTM_STATE_STOPPED;
                                }
                            break;

                            case VTM_STATE_REVERSE:
                                //str_vtm_state = "Reverse";
                                if ( (mySpeed <= 0) ){   //moving backwards //&& (throttleValue < (default_throttle - deadBand))
                                    myCurrent = map (throttleValue, 0, default_throttle, motor_min_current, 0);
                                    UART.setCurrent(myCurrent);
                                }
                                //Stopping
                                if ((mySpeed >= 0) && (throttleValue > default_throttle)){ vtmState = VTM_STATE_STOPPED; }
                            break;

                                /*
                                //Braking while moving backwards
                                if ( (mySpeed < -regen_brake_min_speed) && (throttleValue > default_throttle) ) {    //moving backwards, regen braking
                                    myCurrent = map (throttleValue, default_throttle, 255, 0, abs(motor_max_brake_current));
                                    UART.setBrakeCurrent(myCurrent);
                                } else if ((mySpeed < 0) && (mySpeed > -regen_brake_min_speed) && (throttleValue > default_throttle)) {  //moving backwards, slow, active braking
                                    myCurrent = map(throttleValue, default_throttle, 255, 0, motor_max_current);
                                    UART.setCurrent(myCurrent);   // 
                                }
                                */
                            
                                //deal with STOPPING state transition? timer?
                                //if (throttleValue == 0 && !isMoving()) { setState(STOPPED);}
                                //if (throttleValue == 255 && !isMoving()) { setState(STOPPED);}

                        }
                    break;

                    case STOPPING: // emergency brake when remote has disconnected  -> don't use regen, only active braking.
                        str_vtm_state = "Emergency brake";
                        myCurrent = map(throttleValue, 0, 255, motor_min_current, motor_max_current);
                        UART.setCurrent(myCurrent);   
                    break;

                }
            break;
            //3
            case VTM_RPM_UART:
//                inverse_speed_direction = false;
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                myRpm = map(throttleValue, 0, 255, -10000, +10000);
                if (throttleValue > (default_throttle - deadBand) && throttleValue < (default_throttle + deadBand)) myRpm = 0;
                UART.setRPM(myRpm);
            break;
            //4
            case VTM_DUTY_UART:
//                inverse_speed_direction = false;
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                myDuty = map(throttleValue, (default_throttle + deadBand), 255, 0, 1);
                if (throttleValue > (default_throttle - deadBand) && throttleValue < (default_throttle + deadBand)) myDuty = 0;
                UART.setDuty(myDuty);
            break;
            //5
            case VTM_REGEN_UART:
//                inverse_speed_direction = false;
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                myCurrent = map(throttleValue, 0, 255, -10, +10);
                if ( (mySpeed >= 0) && (throttleValue > (default_throttle + deadBand)) ) {  //going forwards
                    UART.setCurrent(myCurrent);               
                }
                else if ( (mySpeed > 0.5) && (throttleValue < (default_throttle - deadBand)) ) {  //going forwards, braking
                    UART.setBrakeCurrent(myCurrent);
                }                
            break;
            //6
            case VTM_HANDBRAKE_UART:
//                inverse_speed_direction = false;
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                myCurrent = map(throttleValue, 0, 255, -10, +10);
                if ( (mySpeed >= 0) && (throttleValue > (default_throttle + deadBand)) ) {  //going forwards
                    UART.setCurrent(myCurrent);               
                }
                else if ( (mySpeed < 0.1) && (throttleValue < (default_throttle - deadBand)) ) {  //going forwards, braking
                    UART.setHandbrake(myHandbrakeCurrent);   // TEST
                }                
            break;
            //7
            case VTM_POS_UART:
//                inverse_speed_direction = false;
                mySmoothedThrottle.add(throttleValue);
                disablePpmThrottleOutput();
                float myPos = map(throttleValue, 0, 255, 1, +359);
                //if (throttleValue > (default_throttle - deadBand) && throttleValue < (default_throttle + deadBand)) myPos = 0;
                if ( throttleValue > (default_throttle + deadBand) ) {  //going forwards
                    Lpos = pow( Lpos + 1 , 1.2);              
                }
                else if ( throttleValue < (default_throttle - deadBand) ) {  //going forwards, braking
                    Lpos = pow(Lpos , -1.2);   // DOESNT WORK 
                }    
                UART.setPos(Lpos); // TEST
            break;


        }
    #endif

    // remember throttle for smooth auto stop
    lastThrottle = throttleValue;

    #ifdef ROADLIGHT_CONNECTED
      //updateBrakeLight();
    #endif


}

void setCruise(uint8_t speed) {
    // UART
    #ifndef FAKE_UART
        switch(THROTTLE_MODE){
            case VTM_NUNCHUCK_UART:
                disablePpmThrottleOutput();
                UART.nunchuck.valueY = 127;
                UART.nunchuck.upperButton = false;
                UART.nunchuck.lowerButton = true;
                UART.setNunchuckValues();
            break;

            case VTM_PPM_PIN_OUT:
            break;

            case VTM_CURRENT_UART:
            break;

            case VTM_RPM_UART:
            break;

            case VTM_DUTY_UART:
            break;

            case VTM_REGEN_UART:
            break;

            case VTM_HANDBRAKE_UART:
            break; 

            case VTM_POS_UART:
            break;

        }
    #endif
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
  //return abs(ratioRpmSpeed * rpm);
  return (ratioRpmSpeed * rpm);
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
    if ( UART.getVescValues(VESC_COMMAND) ) {
      //float dutyCycleNow;
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
void setDefaultEEPROMSettings(){ //Stores board config variables into boardConfig. packet and calls updateEEPROMSettings()

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

int getSettingValue(uint8_t index){//TODO     // Get settings value by index (useful when iterating through settings).
  // int value;
  // switch (index) {
  //   case 0: value = rxSettings.triggerMode; break;
  //   case 1: value = rxSettings.controlMode; break;
  //
  //   default: return -1;
  // }
  // return value;
}


//***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********

// Set a value of a specific setting in the localOptParamValueArray[] & updates the flash memory.
void setOptParamValue(uint8_t myGlobalSettingIndex, float value){ 
    localOptParamValueArray[myGlobalSettingIndex] = value;
    saveFlashSetting(myGlobalSettingIndex, value);//save to flash memory
}

 // Get a setting value by index from the localOptParamValueArray[]
float getOptParamValue(uint8_t myGlobalSettingIndex){
    float value = localOptParamValueArray[myGlobalSettingIndex];
    return value;
}

/*
void sendOptParamToRemote(uint8_t myGlobalSettingIndex){
    uint8_t arrayIndex = myGlobalSettingIndex;
    //setOptParamValue(myOptIndex, myLightSettingValue);  //store the value locally
    remPacket.command = OPT_PARAM_MODE; //prepare the next packet to update receiver's value
    remPacket.optParamCommand = SET_OPT_PARAM_VALUE;
    remPacket.optParamIndex = arrayIndex;
    remPacket.packOptParamValue(getOptParamValue(arrayIndex));
    requestSendOptParamPacket = true;    //send the value to the receiver
}

void loadOptParamFromRemote(uint8_t myGlobalSettingIndex){
    uint8_t arrayIndex = myGlobalSettingIndex;
    //setOptParamValue(myOptIndex, myLightSettingValue);  //store the value locally
    remPacket.command = OPT_PARAM_MODE; //prepare the next packet to update receiver's value
    remPacket.optParamCommand = GET_OPT_PARAM_VALUE;
    remPacket.optParamIndex = arrayIndex;
    remPacket.packOptParamValue(0); //(getOptParamValue(arrayIndex));
    requestSendOptParamPacket = true;    //send the value to the receiver
}
*/

// Update all local variables from the localOptParamValueArray[] values
void updateOptParamVariables(){
    //LED_BRIGHTNESS_BACK = (int) round(getOptParamValue(IDX_LED_BRIGHTNESS_BACK)); //localOptParamValueArray[1];
    //LED_BRIGHTNESS_FRONT = getOptParamValue(IDX_LED_BRIGHTNESS_FRONT);
    //LED_BRIGHTNESS_BACK = getOptParamValue(IDX_LED_BRIGHTNESS_BACK);
    //LED_BRIGHTNESS_BRAKE = getOptParamValue(IDX_LED_BRIGHTNESS_BRAKE);
    AUTO_CRUISE_ON = getOptParamValue(IDX_AUTO_CRUISE_ON);
    PUSHING_SPEED = getOptParamValue(IDX_PUSHING_SPEED);
    PUSHING_TIME = getOptParamValue(IDX_PUSHING_TIME);
    CRUISE_CURRENT_SPIKE = getOptParamValue(IDX_CRUISE_CURRENT_SPIKE);
    AUTO_CRUISE_TIME = getOptParamValue(IDX_AUTO_CRUISE_TIME);
    CRUISE_CURRENT_LOW = getOptParamValue(IDX_CRUISE_CURRENT_LOW);
    MAX_PUSHING_SPEED = getOptParamValue(IDX_MAX_PUSHING_SPEED);
    AUTO_BRAKE_TIME = getOptParamValue(IDX_AUTO_BRAKE_TIME);
    AUTO_BRAKE_RELEASE = getOptParamValue(IDX_AUTO_BRAKE_RELEASE);
    AUTO_BRAKE_ABORT_MAXSPEED = getOptParamValue(IDX_AUTO_BRAKE_ABORT_MAXSPEED);
    UART_SPEED = getOptParamValue(IDX_UART_SPEED);
    uartPullInterval = getOptParamValue(IDX_uartPullInterval);
    UART_TIMEOUT = getOptParamValue(IDX_UART_TIMEOUT);
    REMOTE_RX_TIMEOUT = getOptParamValue(IDX_REMOTE_RX_TIMEOUT);
    REMOTE_RADIOLOOP_DELAY = getOptParamValue(IDX_REMOTE_RADIOLOOP_DELAY);
    REMOTE_LOCK_TIMEOUT = getOptParamValue(IDX_REMOTE_LOCK_TIMEOUT);
    REMOTE_SLEEP_TIMEOUT = getOptParamValue(IDX_REMOTE_SLEEP_TIMEOUT);
    DISPLAY_BATTERY_MIN = getOptParamValue(IDX_DISPLAY_BATTERY_MIN);
    MOTOR_MIN = getOptParamValue(IDX_MOTOR_MIN);
    MOTOR_MAX = getOptParamValue(IDX_MOTOR_MAX);
    BATTERY_MIN = getOptParamValue(IDX_BATTERY_MIN);
    BATTERY_MAX = getOptParamValue(IDX_BATTERY_MAX);
    MAX_SPEED = getOptParamValue(IDX_MAX_SPEED);
    MAX_RANGE = getOptParamValue(IDX_MAX_RANGE);
    BATTERY_CELLS = getOptParamValue(IDX_BATTERY_CELLS);
    BATTERY_TYPE = getOptParamValue(IDX_BATTERY_TYPE);
    MOTOR_POLES = getOptParamValue(IDX_MOTOR_POLES);
    WHEEL_DIAMETER = getOptParamValue(IDX_WHEEL_DIAMETER);
    WHEEL_PULLEY = getOptParamValue(IDX_WHEEL_PULLEY);
    MOTOR_PULLEY = getOptParamValue(IDX_MOTOR_PULLEY);
    LED_BRIGHTNESS_FRONT = getOptParamValue(IDX_LED_BRIGHTNESS_FRONT);
    LED_BRIGHTNESS_BACK = getOptParamValue(IDX_LED_BRIGHTNESS_BACK);
    LED_BRIGHTNESS_BRAKE = getOptParamValue(IDX_LED_BRIGHTNESS_BRAKE);
    LED_BRIGHTNESS_OFF = getOptParamValue(IDX_LED_BRIGHTNESS_OFF);
    LED_ROADLIGHT_MODE = getOptParamValue(IDX_LED_ROADLIGHT_MODE);
    THROTTLE_MODE = getOptParamValue(IDX_THROTTLE_MODE);
    LIMITED_SPEED_MAX = getOptParamValue(IDX_LIMITED_SPEED_MAX);
}

    //random thought : keep an array of pointers to parameters addresses ( --> iteration )
    /*
    float* localVarAddresses[] //array of pointers
    localVarAddresses[IDX_LED_BRIGHTNESS_FRONT] = &LED_BRIGHTNESS_FRONT     //store local variable addresses in an array
    *localValAddresses[IDX_LED_BRIGHTNESS_FRONT] = (dereference) value stored at contained address      
    */
//***********  VERSION 3 : OPT_PARAM Tx <-> Rx  ***********

//  ######## Settings Flash Storage - ESP32 ########

#include <sstream>
#include <string.h>
//using namespace std;

// Load a setting (index & value)pair from flash memory and update the localOptParamValueArray[] value. Returns the setting (float) value. 
float loadFlashSetting(uint8_t myGlobalSettingIndex, float defaultValue){
    float value;
    stringstream strs;       //convert an int into a char[]
    strs << myGlobalSettingIndex;
    string temp_str = strs.str();
    char const* pchar = temp_str.c_str(); //dont use cast
    receiverPreferences.begin("FireFlyNano", false);
    localOptParamValueArray[myGlobalSettingIndex] = receiverPreferences.getFloat(pchar, defaultValue);
    value = receiverPreferences.getFloat(pchar, defaultValue);
    receiverPreferences.end();
    return value;
}

// Save a setting (index & value)pair into flash memory
void saveFlashSetting(uint8_t myGlobalSettingIndex, float value){
    stringstream strs;       //convert an int into a char[]
    strs << myGlobalSettingIndex;
    string temp_str = strs.str();
    char const* pchar = temp_str.c_str(); //dont use cast
    receiverPreferences.begin("FireFlyNano", false);
    receiverPreferences.putFloat(pchar, localOptParamValueArray[myGlobalSettingIndex]);
    receiverPreferences.end();
}

// SETTINGS INITIALIZATION - copy flash data into local variables & into localOptParamValueArray[] . If nothing saved in flash, GLOBALS.H hardcoded default values are used instead
void refreshAllSettingsFromFlashData(){
  /*    --> these settings are updated via setDefaultEEPROMSettings(){} , stored into boardConfig. packet & changed by remote's calibration page
    MIN_HALL = loadFlashSetting(IDX_MIN_HALL, (float) MIN_HALL);
    CENTER_HALL = loadFlashSetting(IDX_CENTER_HALL, (float) CENTER_HALL);
    MAX_HALL = loadFlashSetting(IDX_MAX_HALL, (float) MAX_HALL);
    BOARD_ID = loadFlashSetting(IDX_BOARD_ID, (float) BOARD_ID);
  */
    AUTO_CRUISE_ON = loadFlashSetting(IDX_AUTO_CRUISE_ON, (float) AUTO_CRUISE_ON);
    PUSHING_SPEED = loadFlashSetting(IDX_PUSHING_SPEED, (float) PUSHING_SPEED);
    PUSHING_TIME = loadFlashSetting(IDX_PUSHING_TIME, (float) PUSHING_TIME);
    CRUISE_CURRENT_SPIKE = loadFlashSetting(IDX_CRUISE_CURRENT_SPIKE, (float) CRUISE_CURRENT_SPIKE);
    AUTO_CRUISE_TIME = loadFlashSetting(IDX_AUTO_CRUISE_TIME, (float) AUTO_CRUISE_TIME);
    CRUISE_CURRENT_LOW = loadFlashSetting(IDX_CRUISE_CURRENT_LOW, (float) CRUISE_CURRENT_LOW);
    MAX_PUSHING_SPEED = loadFlashSetting(IDX_MAX_PUSHING_SPEED, (float) MAX_PUSHING_SPEED);
    AUTO_BRAKE_TIME = loadFlashSetting(IDX_AUTO_BRAKE_TIME, (float) AUTO_BRAKE_TIME);
    AUTO_BRAKE_RELEASE = loadFlashSetting(IDX_AUTO_BRAKE_RELEASE, (float) AUTO_BRAKE_RELEASE);
    AUTO_BRAKE_ABORT_MAXSPEED = loadFlashSetting(IDX_AUTO_BRAKE_ABORT_MAXSPEED, (float) AUTO_BRAKE_ABORT_MAXSPEED);
    UART_SPEED = loadFlashSetting(IDX_UART_SPEED, (float) UART_SPEED);
    uartPullInterval = loadFlashSetting(IDX_uartPullInterval, (float) uartPullInterval);
    UART_TIMEOUT = loadFlashSetting(IDX_UART_TIMEOUT, (float) UART_TIMEOUT);
    REMOTE_RX_TIMEOUT = loadFlashSetting(IDX_REMOTE_RX_TIMEOUT, (float) REMOTE_RX_TIMEOUT);
    REMOTE_RADIOLOOP_DELAY = loadFlashSetting(IDX_REMOTE_RADIOLOOP_DELAY, (float) REMOTE_RADIOLOOP_DELAY);
    REMOTE_LOCK_TIMEOUT = loadFlashSetting(IDX_REMOTE_LOCK_TIMEOUT, (float) REMOTE_LOCK_TIMEOUT);
    REMOTE_SLEEP_TIMEOUT = loadFlashSetting(IDX_REMOTE_SLEEP_TIMEOUT, (float) REMOTE_SLEEP_TIMEOUT);
    DISPLAY_BATTERY_MIN = loadFlashSetting(IDX_DISPLAY_BATTERY_MIN, (float) DISPLAY_BATTERY_MIN);
    MOTOR_MIN = loadFlashSetting(IDX_MOTOR_MIN, (float) MOTOR_MIN);
    MOTOR_MAX = loadFlashSetting(IDX_MOTOR_MAX, (float) MOTOR_MAX);
    BATTERY_MIN = loadFlashSetting(IDX_BATTERY_MIN, (float) BATTERY_MIN);
    BATTERY_MAX = loadFlashSetting(IDX_BATTERY_MAX, (float) BATTERY_MAX);
    MAX_SPEED = loadFlashSetting(IDX_MAX_SPEED, (float) MAX_SPEED);
    MAX_RANGE = loadFlashSetting(IDX_MAX_RANGE, (float) MAX_RANGE);
    BATTERY_CELLS = loadFlashSetting(IDX_BATTERY_CELLS, (float) BATTERY_CELLS);
    BATTERY_TYPE = loadFlashSetting(IDX_BATTERY_TYPE, (float) BATTERY_TYPE);
    MOTOR_POLES = loadFlashSetting(IDX_MOTOR_POLES, (float) MOTOR_POLES);
    WHEEL_DIAMETER = loadFlashSetting(IDX_WHEEL_DIAMETER, (float) WHEEL_DIAMETER);
    WHEEL_PULLEY = loadFlashSetting(IDX_WHEEL_PULLEY, (float) WHEEL_PULLEY);
    MOTOR_PULLEY = loadFlashSetting(IDX_MOTOR_PULLEY, (float) MOTOR_PULLEY);
    LED_BRIGHTNESS_FRONT = loadFlashSetting(IDX_LED_BRIGHTNESS_FRONT, (float) LED_BRIGHTNESS_FRONT);
    LED_BRIGHTNESS_BACK = loadFlashSetting(IDX_LED_BRIGHTNESS_BACK, (float) LED_BRIGHTNESS_BACK);
    LED_BRIGHTNESS_BRAKE = loadFlashSetting(IDX_LED_BRIGHTNESS_BRAKE, (float) LED_BRIGHTNESS_BRAKE);
    LED_BRIGHTNESS_OFF = loadFlashSetting(IDX_LED_BRIGHTNESS_OFF, (float) LED_BRIGHTNESS_OFF);
    LED_ROADLIGHT_MODE = loadFlashSetting(IDX_LED_ROADLIGHT_MODE, (float) LED_ROADLIGHT_MODE);
    THROTTLE_MODE = loadFlashSetting(IDX_THROTTLE_MODE, (float) THROTTLE_MODE);
    LIMITED_SPEED_MAX = loadFlashSetting(IDX_LIMITED_SPEED_MAX, (float) LIMITED_SPEED_MAX);
//     = loadFlashSetting(IDX_, (float) );
}
//  ######## Settings Flash Storage - ESP32 ########



bool inRange(int val, int minimum, int maximum){ //checks if value is within MIN - MAX range
  return ((minimum <= val) && (val <= maximum));
}

// **************************************** LED ROADLIGHTS *****************************
//void drawLightPage(); // uint8_t lightBrightnessValue

#ifdef ROADLIGHT_CONNECTED
    void switchLightOn(){
        ledcWrite(led_pwm_channel_frontLight, LED_BRIGHTNESS_FRONT);
        ledcWrite(led_pwm_channel_backLight, LED_BRIGHTNESS_BACK);
        myRoadLightState = ON;
    }

    void switchLightOff(){
        ledcWrite(led_pwm_channel_frontLight, LED_BRIGHTNESS_OFF);
        ledcWrite(led_pwm_channel_backLight, LED_BRIGHTNESS_OFF);
        myRoadLightState = OFF;
    }

    void switchLightBrakesOnly(){
        ledcWrite(led_pwm_channel_frontLight, LED_BRIGHTNESS_OFF);
        ledcWrite(led_pwm_channel_backLight, LED_BRIGHTNESS_OFF);
        myRoadLightState = BRAKES_ONLY;
    }


    void updateBrakeLight(){   // activate BACKLIGHT flashing sequence from setThrottle() function
        switch(myRoadLightState){
            case OFF:
                // do nothing
            break;

            case ON:
                emitBrakeLightPulse(LED_BRIGHTNESS_BACK); //activate brakeLight flashes while ON (normal brightness) in between
            break;

            case BRAKES_ONLY:
                emitBrakeLightPulse(LED_BRIGHTNESS_OFF); //activate brakeLight flashes while OFF in between
            break;
        }
    }

    void emitBrakeLightPulse(uint_fast32_t dutyCycle_returnToNormal){    //emits a brakeLight flash and go back to the previous state
        uint8_t pwm_channel = led_pwm_channel_backLight;
        uint_fast32_t returnDutyCycle = dutyCycle_returnToNormal;
        uint_fast32_t flashDutyCycle = LED_BRIGHTNESS_BRAKE;

            if (millisSince(lastBrakeLightPulse) >= brakeLightPulseInterval) {    // check when was the last brake flash triggered
                if(lastThrottle<(default_throttle*0.75)){
                    lastBrakeLightPulse = millis(); //reset coounter
                    ledcWrite(pwm_channel, flashDutyCycle);     //emit a new flash
                }
            }
            else{ //during a brakeLightPulse emission
                if(millisSince(lastBrakeLightPulse) >= brakeLightPulseDuration){ // check if the flash has been ON long enough
                            ledcWrite(pwm_channel, returnDutyCycle); // go back to previous state
                }
            }
             //debug unexpected state
            if(lastThrottle >= (default_throttle*0.98)){ //debug unexpected state
                lastBrakeLightPulse = millis(); //reset coounter
                ledcWrite(pwm_channel, returnDutyCycle);     //emit a new flash
            }
    }

#endif
// **************************************** LED ROADLIGHTS *****************************

// ******** PPM THROTTLE OUTPUT ********
#ifdef OUTPUT_PPM_THROTTLE
    void updatePpmThrottleOutput(int myThrottle){
//        uint_fast32_t pwm_throttle_dutyCycle_value = map(myThrottle, 0, 255, 3276, 6552); //throttle; //map(throttle, 0, 255, 1ms, 2ms);
        uint_fast32_t pwm_throttle_dutyCycle_value = map(myThrottle, 0, 255, 3276, 6552); //throttle; //map(throttle, 0, 255, 1ms, 2ms);
        ledcWrite(pwm_throttle_channel, pwm_throttle_dutyCycle_value);
    }
#endif

void disablePpmThrottleOutput(){
    #ifdef OUTPUT_PPM_THROTTLE
        ledcWrite(pwm_throttle_channel, 0);
    #endif
}
// ******** PPM THROTTLE OUTPUT ********


/*
int smoothValueOverTime(int valueToAdd){
    int myArraySize = sizeof(arraySmoothValue)/sizeof(arraySmoothValue[0]);
    int samples = myArraySize;
    int total;
    for (uint8_t i = 0; i < samples; i++) {

            if (millisSince(smoothTimestamp) > 15){
                total = 0;
                samples = 0;
                //totalAccumulatedSmoothValues ++;
                for (i=0; i < (myArraySize-1); i++){
                    arraySmoothValue[i] = arraySmoothValue[i+1]; 
                    total = total + arraySmoothValue[i];
                    samples ++;                               
                }
                arraySmoothValue[(myArraySize-1)] = valueToAdd;
                total = total + valueToAdd;
                samples++;
                myAverageValue = (int)(total / samples);
                smoothTimestamp = millis();
            }
    }
    return myAverageValue;
}
*/

/*
float smoothValue2(float *smoothArray2, float valueToAdd){
    int myArraySize = sizeof(smoothArray2)/sizeof(smoothArray2[0]);
    int samples = myArraySize;
    float total;
    float myAverageValue;
    for (uint8_t i = 0; i < samples; i++) {
            if (millisSince(smoothTimestamp2) > 15){
                total = 0;
                samples = 0;
                for (i=0; i < (myArraySize-1); i++){
                    smoothArray2[i] = smoothArray2[i+1]; 
                    total = total + smoothArray2[i];
                    samples ++;                               
                }
                smoothArray2[(myArraySize-1)] = valueToAdd;
                total = total + valueToAdd;
                samples++;
                myAverageValue = (float)(total / samples);
                smoothTimestamp2 = millis();
            }
            else {
                total = 0;
                for (i=0; i < (myArraySize); i++){
                    total = total + smoothArray2[i];                            
                }
                myAverageValue = (float)total/myArraySize;
                
            }
    }
    return myAverageValue;
}*/