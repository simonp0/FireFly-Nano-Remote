

void throttle_app(uint16_t throttleValue){

//throttle mode :
//- VESC Remote - Nunchuk
//- Current
//- PPM
//- ?

//brake mode :
//- VESC Remote - Nunchuk
//- setBrake (regen)
//- PPM
//- ?
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
int mySpeed=speed();

bool enable_activeBraking;
bool enable_handBrake;
bool enable_reverse;
enum AppThrottleMode {NUNCHUK, CURRENT, PPM} app_throttle_mode = CURRENT;

case VTM_CURRENT_UART:  //current with regen brake & handbrake when stopped
                //myCurrent = map(throttleValue, 0, 255, -abs(motor_max_brake_current), motor_max_current);

                switch (state){
                    case CONNECTED:
                        switch (vtmState){

                            case VTM_STATE_STOPPED:
                                str_vtm_state = "Stopped";
                                //Idle
                                if (throttleValue > (default_throttle - deadBand) && throttleValue < (default_throttle + deadBand)){ //IDLE - remote centered
                                    switch(app_throttle_mode){
                                            case NUNCHUK:
                                                UART.nunchuck.valueY = throttleValue;
                                                UART.nunchuck.upperButton = false;
                                                UART.nunchuck.lowerButton = false;
                                                UART.setNunchuckValues();
                                            break;
                                            case CURRENT:
                                                myCurrent = 0;
                                                UART.setBrakeCurrent(myCurrent);                                                
                                            break;
                                    }                                 
                                } 

                                //Start going forward from stop                        
                                if ((mySpeed >= (-stoppedStateMaxSpeed)) && (throttleValue > (default_throttle + deadBand) )) {  //moving forwards, cruising
                                        switch(app_throttle_mode){
                                                case NUNCHUK:
                                                    UART.nunchuck.valueY = throttleValue;
                                                    UART.nunchuck.upperButton = false;
                                                    UART.nunchuck.lowerButton = false;
                                                    UART.setNunchuckValues();
                                                break;
                                                case CURRENT:
                                                    myCurrent = map(throttleValue, default_throttle, 255, 0, motor_max_current);
                                                    UART.setCurrent(myCurrent);
                                                break;
                                        }                                    
                                    vtmState = VTM_STATE_DRIVING;               
                                }
                                // Start rolling forward without throttle input
                                if (mySpeed > stoppedStateMaxSpeed ){ vtmState = VTM_STATE_DRIVING;}

                                //Start going backwards from stop
                                if ( (mySpeed <= stoppedStateMaxSpeed) && (throttleValue < (default_throttle - deadBand)) ){   //moving backwards
                                    if (enable_reverse){
                                        switch(app_throttle_mode){
                                                case NUNCHUK:
                                                    UART.nunchuck.valueY = throttleValue;
                                                    UART.nunchuck.upperButton = false;
                                                    UART.nunchuck.lowerButton = false;
                                                    UART.setNunchuckValues();
                                                break;
                                                case CURRENT:
                                                    myCurrent = map (throttleValue, 0, default_throttle, motor_min_current, 0);
                                                    UART.setCurrent(myCurrent);                                                
                                                break;
                                        }
                                        vtmState = VTM_STATE_REVERSE;
                                    }  
                                    else{
                                        switch(app_throttle_mode){
                                                case NUNCHUK:
                                                    UART.nunchuck.valueY = throttleValue;
                                                    UART.nunchuck.upperButton = false;
                                                    UART.nunchuck.lowerButton = false;
                                                    UART.setNunchuckValues();
                                                break;
                                                case CURRENT:
                                                    myCurrent = map (throttleValue, 0, default_throttle, motor_min_current, 0);
                                                    UART.setBrakeCurrent(myCurrent);                                                
                                                break;
                                        }
                                        vtmState = VTM_STATE_REVERSE;
                                        //brake
                                    }

                                }
                                // Start rolling backward without throttle input
                                if (mySpeed < (-stoppedStateMaxSpeed) ){ vtmState = VTM_STATE_REVERSE;}

                            break;

                            case VTM_STATE_DRIVING:
                                
                                //Keep driving, update motor throttle.
                                if ( (mySpeed >= (-handbrakeMaxSpeed)) && (throttleValue > default_throttle) ) {  //moving forwards, cruising
                                    switch(app_throttle_mode){
                                            case NUNCHUK:
                                                UART.nunchuck.valueY = throttleValue;
                                                UART.nunchuck.upperButton = false;
                                                UART.nunchuck.lowerButton = false;
                                                UART.setNunchuckValues();
                                            break;
                                            case CURRENT:
                                                myCurrent = map(throttleValue, default_throttle, 255, 0, motor_max_current);
                                                UART.setCurrent(myCurrent);
                                            break;
                                    }                                    
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

                                
                                //Braking while moving backwards
                            //    if ( (mySpeed < -regen_brake_min_speed) && (throttleValue > default_throttle) ) {    //moving backwards, regen braking
                            //        myCurrent = map (throttleValue, default_throttle, 255, 0, abs(motor_max_brake_current));
                            //        UART.setBrakeCurrent(myCurrent);
                            //    } else if ((mySpeed < 0) && (mySpeed > -regen_brake_min_speed) && (throttleValue > default_throttle)) {  //moving backwards, slow, active braking
                            //        myCurrent = map(throttleValue, default_throttle, 255, 0, motor_max_current);
                            //        UART.setCurrent(myCurrent);   // 
                            //    }
                                
                            
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




class RemoteButton{

    //by default : private
    int privateNumber;  

    public:
        int number;

        //CONSTRUCTOR
        RemoteButton(int inputPin){           
        }

        //DESTRUCTOR
        ~RemoteButton(){
        }

        int inlineFunction(){   //inline by default when defined inside a class
            return number;
        }
        int checkButtonState();

        int getNumber(){
            return number;
        }

        void setNumber(int value){}
}

// member function defined outside class definition
int RemoteButton::checkButtonState(){
    //big functions -> define outside class to avoid being an inline function
    int event = 0;
    buttonVal = digitalRead(PIN_PWRBUTTON);

    // Button pressed down
    if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce) {
        downTime = millis();
        ignoreUp = false;
        waitForUp = false;
        singleOK = true;
        holdEventPast = false;
        longHoldEventPast = false;
        if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true){
            DConUp = true;
        }
        else {  DConUp = false; }
        DCwaiting = false;
    }
    // Button released
    else {
        if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce){
            if (not ignoreUp){
                upTime = millis();
                if (DConUp == false){
                    DCwaiting = true;
                }
                else{
                    event = DBL_CLICK;
                    DConUp = false;
                    DCwaiting = false;
                    singleOK = false;
                }
            }
        }
    }

    // Test for normal click event: DCgap expired
    if ( buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2){
        event = CLICK;
        DCwaiting = false;
    }
    // Test for hold
    if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
        // Trigger "normal" hold
        if (not holdEventPast){
            event = HOLD;
            waitForUp = true;
            ignoreUp = true;
            DConUp = false;
            DCwaiting = false;
            holdEventPast = true;
        }
        // Trigger "long" hold
        if ((millis() - downTime) >= longHoldTime){
            if (not longHoldEventPast){
                event = LONG_HOLD;
                longHoldEventPast = true;
            }
        }
    }

    buttonLast = buttonVal;
    return event;    
}

/*
// Button constants
const int RELEASED  = 0;
const int CLICK     = 1;
const int DBL_CLICK = 2;
const int HOLD      = 3;
const int LONG_HOLD = 4;

enum class ButtonState {
    RELEASED,
    CLICK,
    DBL_CLICK,
    HOLD,
    LONG_HOLD
} PwrButtonState = ButtonState::RELEASED;

// Button timing variables
int debounce = 50;          // ms debounce period to prevent flickering when pressing or releasing the button (20ms)
int DCgap = 250;            // max ms between clicks for a double click event
int holdTime = 300;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 1000;    // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already
*/





class RemoteButton{
    //by default : private
    int privateNumber;  
    public:
        int number;
        //CONSTRUCTOR
        RemoteButton(int inputPin){           
        }
        //DESTRUCTOR
        ~RemoteButton(){
        }
        int inlineFunction(){   //inline by default when defined inside a class
            return number;
        }
        int memberFunction();
        int getNumber(){
            return number;
        }
        void setNumber(int value){}
}
// member function defined outside class definition
int RemoteButton::memberFunction(){
    //big functions -> define outside class to avoid being an inline function
}