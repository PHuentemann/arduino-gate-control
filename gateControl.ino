#include <Adafruit_INA219.h>
#include <Wire.h>

Adafruit_INA219 ina219_left;
Adafruit_INA219 ina219_right(0x41);

int pinRemoteSwitch = 2;
int pinLightSwitch = 3;
int pinKeypad = 4;

int pinLeftOpen = 5;
int pinLeftClose = 6;
int pinRightOpen = 7;
int pinRightClose = 8;
int pinLight = 9;


int pinPot1 = 0;
int pinPot2 = 1;

bool remoteState = false; 
bool oldRemoteState = false; 
bool lightswitchState = false; 
bool oldLightswitchState = false; 
bool keypadState = false;
bool oldKeypadState = false;

bool gateOpening = false;
bool gateClosing = false;
bool gateState = false;

unsigned long openTime = 0;
unsigned long closeTime = 0;
unsigned long lightTime = 0;

bool lightState = false;

bool openLeftStarted = false;
bool openLeftFinished = false;
bool openRightStarted = false;

bool closeLeftStarted = false;
bool closeRightStarted = false;
bool closeRightFinished = false;

int timeOffset = 3000;
int timeRuntime = 13000;


void setup() {
    uint32_t currentFrequency;

    pinMode(pinRemoteSwitch, INPUT_PULLUP);
    pinMode(pinLightSwitch, INPUT_PULLUP);
    pinMode(pinKeypad, INPUT_PULLUP);

    pinMode(pinLeftOpen, OUTPUT);
    pinMode(pinLeftClose, OUTPUT);
    pinMode(pinRightOpen, OUTPUT);
    pinMode(pinRightClose, OUTPUT);
    pinMode(pinLight, OUTPUT);

    pinMode(pinPot1, INPUT);
    pinMode(pinPot2, INPUT);

    digitalWrite(pinLeftOpen, HIGH);
    digitalWrite(pinLeftClose, HIGH);
    digitalWrite(pinRightOpen, HIGH);
    digitalWrite(pinRightClose, HIGH);
    digitalWrite(pinLight, HIGH);

    Serial.begin(9600);

    if (! ina219_left.begin()) {
        Serial.println("Failed to find INA219 chip for left gate motor");
        while (1) { delay(50); }
    }
    if (! ina219_right.begin()) {
        Serial.println("Failed to find INA219 chip for right gate motor");
        while (1) { delay(50); }
    }
}

void loop() {
    float leftCurrent = 0;
    float rightCurrent = 0;

    leftCurrent = ina219_left.getCurrent_mA();
    rightCurrent = ina219_right.getCurrent_mA();

    float pot1 = fmap(analogRead(pinPot1), 0, 1023, 0.0, 5.0);
    float pot2 = fmap(analogRead(pinPot2), 0, 1023, 0.0, 5.0);
    
    remoteState = (digitalRead(pinRemoteSwitch) == LOW);
    lightswitchState = (digitalRead(pinLightSwitch) == LOW);
    keypadState = (digitalRead(pinKeypad) == LOW);

    if ((remoteState || keypadState) && (!gateOpening) && (!gateClosing)) {
        if (gateState) {
            gateClosing = true;
        } else {
            gateOpening = true;
        }
    }

    if (lightswitchState && (!gateOpening)) {
        if (gateClosing) {
            digitalWrite(pinRightClose, HIGH);
            digitalWrite(pinLeftClose, HIGH);
            delay(2000);
            closeLeftStarted = false;
            closeRightStarted = false;
            closeRightFinished = false;
            gateClosing = false;
            gateState = false;
            gateOpening = true;
        } else if (!gateState) {
            gateOpening = true;
        }
    }

    if (gateOpening) {

        if (!lightState) {
            lightTime = millis();
            digitalWrite(pinLight, LOW);
            lightState = true;
        } else {
            if ((millis() - lightTime) >= 1000) {
                digitalWrite(pinLight, HIGH);
            }
            if ((millis() - lightTime) >= 2000) {
                lightState = false;
            }
        }
        
        if (!openLeftStarted) {
            Serial.println("Opening Left Gate - Start");
            digitalWrite(pinLeftOpen, LOW);
            openTime = millis();
            openLeftStarted = true;
        } else {
            if (!openRightStarted) {
                if ((millis() - openTime) >= timeOffset) {
                    //Serial.println(millis() - openTime);
                    Serial.println("Opening Right Gate - Start");
                    digitalWrite(pinRightOpen, LOW);
                    openRightStarted = true;
                }
            } else {
                if (((millis() - openTime) >= timeRuntime) && (!openLeftFinished)) {
                    //Serial.println(millis() - openTime);
                    Serial.println("Opening Left Gate - Stop");
                    digitalWrite(pinLeftOpen, HIGH);
                    openLeftFinished = true;
                }
                if ((millis() - openTime) >= (timeRuntime + timeOffset)) {
                    //Serial.println(millis() - openTime);
                    Serial.println("Opening Right Gate - Stop");
                    digitalWrite(pinRightOpen, HIGH);
                    digitalWrite(pinLight, HIGH);
                    openLeftStarted = false;
                    openLeftFinished = false;
                    openRightStarted = false;
                    gateOpening = false;
                    gateState = true;
                    Serial.println("Gate is now opened!");
                    delay(1000);
                }
            }
        }
    }

    if (gateClosing) {

        if (!lightState) {
            lightTime = millis();
            digitalWrite(pinLight, LOW);
            lightState = true;
        } else {
            if ((millis() - lightTime) >= 1000) {
                digitalWrite(pinLight, HIGH);
            }
            if ((millis() - lightTime) >= 2000) {
                lightState = false;
            }
        }

        if (!closeLeftStarted) {
            Serial.println("Closing Right Gate - Start");
            digitalWrite(pinRightClose, LOW);
            closeTime = millis();
            closeLeftStarted = true;
        } else {
            if (!closeRightStarted) {
                if ((millis() - closeTime) >= timeOffset) {
                    //Serial.println(millis() - closeTime);
                    Serial.println("Closing Left Gate - Start");
                    digitalWrite(pinLeftClose, LOW);
                    closeRightStarted = true;
                }
            } else {
                if (((millis() - closeTime) >= timeRuntime) && (!closeRightFinished)) {
                    //Serial.println(millis() - closeTime);
                    Serial.println("Closing Right Gate - Stop");
                    digitalWrite(pinRightClose, HIGH);
                    closeRightFinished = true;
                }
                if ((millis() - closeTime) >= (timeRuntime + timeOffset)) {
                    //Serial.println(millis() - closeTime);
                    Serial.println("Closing Left Gate - Stop");
                    digitalWrite(pinLeftClose, HIGH);
                    digitalWrite(pinLight, HIGH);
                    closeLeftStarted = false;
                    closeRightStarted = false;
                    closeRightFinished = false;
                    gateClosing = false;
                    gateState = false;
                    Serial.println("Gate is now closed!");
                    delay(1000);
                }
            }
        }
    }

}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}