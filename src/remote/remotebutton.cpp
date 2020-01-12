#include <Arduino.h>
//#include <CPU.h>
//#include <globals.h>
#include <utils.h>
//#include <iostream>
//using namespace std;

    // Button timing variables
//    int debounce = 50;          // ms debounce period to prevent flickering when pressing or releasing the button (20ms)
//    int DCgap = 250;            // max ms between clicks for a double click event
//    int holdTime = 300;        // ms hold period: how long to wait for press+hold event
//    int longHoldTime = 1000;    // ms long hold period: how long to wait for press+hold event


class RemoteButton{

    //by default : private
    private:


    public:
        int inputPin;
        int debounce;
        int DCgap;
        int holdTime;
        int longHoldTime;
        int memoryDelay;

        // Button variables
        boolean buttonValue = HIGH;   // value read from button
        boolean buttonLast = HIGH;  // buffered value of the button's previous state
        boolean DCwaiting = false;  // whether we're waiting for a double click (down)
        boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
        boolean singleOK = true;    // whether it's OK to do a single click
        long downTime = -1;         // time the button was pressed down
        long upTime = -1;           // time the button was released
        boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
        boolean waitForUp = false;        // when held, whether to wait for the up event
        boolean longHoldEventPast = false;// whether or not the long hold event happened already
        boolean holdEventPast = false;    // whether or not the hold event happened already

        long lastStateTimestamp = 0; 
        int currentButtonState = 0;        

        //CONSTRUCTOR
        //RemoteButton(){}
        RemoteButton(int inputPin, int debounce, int DCgap, int holdTime, int longHoldTime, int memoryDelay){  
            this->inputPin = inputPin;     
            this->debounce = debounce;
            this->DCgap = DCgap;
            this->holdTime = holdTime;
            this->longHoldTime = longHoldTime;
            this->memoryDelay = memoryDelay;    
        }

        //DESTRUCTOR
        ~RemoteButton(){
        }


        enum ButtonState {
            RELEASED,
            CLICK,
            DBL_CLICK,
            HOLD,
            LONG_HOLD
        } myState = ButtonState::RELEASED;
        
        //void checkButtonState();
//int checkButtonState(){

        void readButtonState(){
            //big functions -> define outside class to avoid being an inline function
            int event = 0;
            buttonValue = digitalRead(inputPin);

            // Button pressed down
            if (buttonValue == LOW && buttonLast == HIGH && (millis() - upTime) > debounce) {
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
                if (buttonValue == HIGH && buttonLast == LOW && (millis() - downTime) > debounce){
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
            if ( buttonValue == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2){
                event = CLICK;
                DCwaiting = false;
            }
            // Test for hold
            if (buttonValue == LOW && (millis() - downTime) >= holdTime) {
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

            buttonLast = buttonValue;
            //return event;
            currentButtonState = event;
        }

        void update(){ //update the button state -> call from main program loop()
            readButtonState();
            switch (currentButtonState) { //checks what PWR_BUTTON is doing and return it's state ( CLICK - DBL_CLICK - HOLD - LONG_HOLD )
                case RELEASED:
                    if (millisSince(lastStateTimestamp) > memoryDelay) {
                        myState = RELEASED;
                        lastStateTimestamp = millis();
                    }
                break;
                case CLICK:
                    myState = CLICK;
                break;
                case DBL_CLICK:
                    myState = DBL_CLICK;
                break;
                case HOLD:
                    myState = HOLD;
                break;
                case LONG_HOLD:
                    myState = LONG_HOLD;
                break;
            }
        }

        int getState(){
            return myState;
        }

};

// member function defined outside class definition
/*
void RemoteButton::checkButtonState(){
   //big functions -> define outside class to avoid being an inline function
    int event = 0;
    buttonValue = digitalRead(PIN_PWRBUTTON);

    // Button pressed down
    if (buttonValue == LOW && buttonLast == HIGH && (millis() - upTime) > debounce) {
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
        if (buttonValue == HIGH && buttonLast == LOW && (millis() - downTime) > debounce){
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
    if ( buttonValue == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2){
        event = CLICK;
        DCwaiting = false;
    }
    // Test for hold
    if (buttonValue == LOW && (millis() - downTime) >= holdTime) {
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

    buttonLast = buttonValue;
    //return event;
    currentButtonState = event;
}
*/
