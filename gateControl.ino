#include <Adafruit_INA219.h>
#include <Wire.h>
#include <EEPROM.h>

Adafruit_INA219 ina219_left;
Adafruit_INA219 ina219_right(0x41);

int pinRemoteSwitch = 2;
int pinLightSwitch = 4;
int pinKeypad = 5;
int pinButton = 6;

int pinLeftOpen = 12;
int pinLeftClose = 11;
int pinRightOpen = 10;
int pinRightClose = 9;
int pinLight = 8;

int pinPot1 = 0;
int pinPot2 = 1;

int remoteState = 0;
int lastRemoteState = 0;
bool remotePulse = false;

int lightswitchState = 0;
int lastLightswitchState = 0;
bool lightswitchPulse = false;

int keypadState = 0;
int lastKeypadState = 0;
bool keypadPulse = false;

int buttonState = 0;
int lastButtonState = 0;
bool buttonPulse = false;

bool calibrateLeft = false;
bool calibrateRight = false;
bool calibrationSide = true;

bool gateCalibration = false;
bool gateOpening = false;
bool gateClosing = false;
bool gateState = false;

unsigned long openLeftTime = 0;
unsigned long openRightTime = 0;
unsigned long closeLeftTime = 0;
unsigned long closeRightTime = 0;
unsigned long lightTime = 0;
unsigned long buttonPressed = 0;
unsigned long leftCalibrationTime = 0;
unsigned long rightCalibrationTime = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;

bool lightState = false;

bool openLeftStarted = false;
bool openRightStarted = false;
bool openLeftFinished = false;

bool closeLeftStarted = false;
bool closeRightStarted = false;
bool closeRightFinished = false;

int leftMaxCurrent = 2500;
int rightMaxCurrent = 2500;
int inrushCurrentDelay = 500;

int calibrationLimit = 25000;
int leftRuntime = 12000;
int rightRuntime = 12000;


void setup() {
    byte leftVal = EEPROM.read(0);
    byte rightVal = EEPROM.read(1);
    if (leftVal != 0) {
        EEPROM.write(0, leftRuntime / 100);
    } else {
        leftRuntime = leftVal * 100;
    }
    if (rightVal != 0) {
        EEPROM.write(0, rightRuntime / 100);
    } else {
        rightRuntime = rightVal * 100;
    }
    
    uint32_t currentFrequency;

    pinMode(pinRemoteSwitch, INPUT_PULLUP);
    pinMode(pinLightSwitch, INPUT_PULLUP);
    pinMode(pinKeypad, INPUT_PULLUP);
    pinMode(pinButton, INPUT_PULLUP);

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
    
    remoteState = digitalRead(pinRemoteSwitch);
    lightswitchState = digitalRead(pinLightSwitch);
    keypadState = digitalRead(pinKeypad);
    buttonState = digitalRead(pinButton);

    remotePulse = ((remoteState == LOW) && (remoteState != lastRemoteState));
    lightswitchPulse = ((remoteState == LOW) && (lightswitchState != lastLightswitchState));
    keypadPulse = ((remoteState == LOW) && (keypadState != lastKeypadState));
    buttonPulse = ((remoteState == LOW) && (buttonState != lastButtonState));

    if (!gateCalibration) {
        if (buttonPulse) {
            if (gateOpening || gateClosing) {
                // Abort everything and stop at current position
                Serial.println("ABORTING PROCESS!!!");
                digitalWrite(pinLeftOpen, HIGH);
                digitalWrite(pinLeftClose, HIGH);
                digitalWrite(pinRightOpen, HIGH);
                digitalWrite(pinRightClose, HIGH);
                digitalWrite(pinLight, HIGH);
                openLeftStarted = false;
                openRightStarted = false;
                openLeftFinished = false;
                gateOpening = false;
                closeLeftStarted = false;
                closeRightStarted = false;
                closeRightFinished = false;
                gateClosing = false;
                gateOpening = false;
                gateState = true;
                delay(1000);
            } else {
                if (buttonState == LOW) {
                    buttonPressed = millis();
                } else {
                    if ((millis() - buttonPressed) > 3000) {
                        if (!gateCalibration) {
                            gateCalibration = true;
                        }
                    } 
                }
            }
        }

        if ((remotePulse || keypadPulse) && (!gateOpening) && (!gateClosing)) {
            if (gateState) {
                gateClosing = true;
            } else {
                gateOpening = true;
            }
        }

        if (lightswitchPulse && (!gateOpening)) {
            if (gateClosing) {
                digitalWrite(pinRightClose, HIGH);
                digitalWrite(pinLeftClose, HIGH);
                delay(1000);
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
            // Switching light relay on and off at 1Hz
            if (!lightState) {
                lightTime = millis();
                digitalWrite(pinLight, LOW);
                lightState = true;
                Serial.println("Left current: " + String(leftCurrent) + " mA");
                Serial.println("Right current: " + String(rightCurrent) + " mA");
            } else {
                if ((millis() - lightTime) >= 1000) {
                    digitalWrite(pinLight, HIGH);
                    Serial.println("Left current: " + String(leftCurrent) + " mA");
                    Serial.println("Right current: " + String(rightCurrent) + " mA");
                }
                if ((millis() - lightTime) >= 2000) {
                    lightState = false;
                }
            }
            // Check if left gate is already opening
            if (!openLeftStarted) {
                Serial.println("Opening Left Gate - Start");
                digitalWrite(pinLeftOpen, LOW);
                openLeftTime = millis();
                openLeftStarted = true;
            } else {
                if (!openRightStarted) {
                    if (!openLeftFinished) {
                        if ((millis() - openLeftTime) >= leftRuntime) {
                            Serial.println("Opening Left Gate - Stop");
                            digitalWrite(pinLeftOpen, HIGH);
                            openLeftFinished = true;
                        }
                    } else {
                        if ((millis() - openLeftTime) >= (leftRuntime + 500)) {
                            Serial.println("Opening Right Gate - Start");
                            digitalWrite(pinRightOpen, LOW);
                            openRightTime = millis();
                            openRightStarted = true;
                        }
                    }
                } else {
                    if ((millis() - openRightTime) >= rightRuntime) {
                        Serial.println("Opening Right Gate - Stop");
                        digitalWrite(pinRightOpen, HIGH);
                        digitalWrite(pinLight, HIGH);
                        openLeftStarted = false;
                        openRightStarted = false;
                        openLeftFinished = false;
                        gateOpening = false;
                        gateState = true;
                        openLeftTime = 0;
                        openRightTime = 0;
                        Serial.println("Gate is now opened!");
                        delay(500);
                    }
                }
            }
        }

        if (gateClosing) {
            // Switching light relay on and off at 1Hz
            if (!lightState) {
                lightTime = millis();
                digitalWrite(pinLight, LOW);
                lightState = true;
                Serial.println("Left current: " + String(leftCurrent) + " mA");
                Serial.println("Right current: " + String(rightCurrent) + " mA");
            } else {
                if ((millis() - lightTime) >= 1000) {
                    digitalWrite(pinLight, HIGH);
                    Serial.println("Left current: " + String(leftCurrent) + " mA");
                    Serial.println("Right current: " + String(rightCurrent) + " mA");
                }
                if ((millis() - lightTime) >= 2000) {
                    lightState = false;
                }
            }
            // Check if right gate is already closing 
            if (!closeRightStarted) {
                Serial.println("Closing Right Gate - Start");
                digitalWrite(pinRightClose, LOW);
                closeRightTime = millis();
                closeRightStarted = true;
            } else {
                if ((rightCurrent < rightMaxCurrent) && ((millis() - closeRightTime) >= inrushCurrentDelay)) {
                    if (!closeLeftStarted) {
                        if(!closeRightFinished) {
                            if ((millis() - closeRightTime) >= rightRuntime) {
                                Serial.println("Closing Right Gate - Stop");
                                digitalWrite(pinRightClose, HIGH);
                                closeRightFinished = true;
                            }    
                        } else {
                            if ((millis() - closeRightTime) >= (rightRuntime + 500)) {
                                Serial.println("Closing Left Gate - Start");
                                digitalWrite(pinLeftClose, LOW);
                                closeLeftTime = millis();
                                closeLeftStarted = true;
                            }
                        }
                    } else {
                        if ((leftCurrent < leftMaxCurrent) && ((millis() - closeLeftTime) >= inrushCurrentDelay)) {
                            if ((millis() - closeLeftTime) >= leftRuntime) {
                                Serial.println("Closing Left Gate - Stop");
                                digitalWrite(pinLeftClose, HIGH);
                                digitalWrite(pinLight, HIGH);
                                closeLeftStarted = false;
                                closeRightStarted = false;
                                gateClosing = false;
                                gateState = false;
                                closeRightTime = 0;
                                closeLeftTime = 0;
                                Serial.println("Gate is now closed!");
                                delay(500);
                            }
                        } else {
                            // Overcurrent protection for left gate
                            Serial.println("Left gate current above 2500mA - Stopping!");
                            digitalWrite(pinLeftClose, HIGH);
                            digitalWrite(pinRightClose, HIGH);
                            closeLeftStarted = false;
                            closeRightStarted = false;
                            gateClosing = false;
                            gateState = false;
                            delay(500);
                        }
                    }
                } else {
                    // Overcurrent protection for right gate
                    Serial.println("Right gate current above 2500mA - Stopping!");
                    digitalWrite(pinRightClose, HIGH);
                    digitalWrite(pinLeftClose, HIGH);
                    closeLeftStarted = false;
                    closeRightStarted = false;
                    gateClosing = false;
                    gateState = false;
                    delay(500);
                }
            }
        }
    } else {
        // Calibration mode
        if (!lightState) {
            lightTime = millis();
            digitalWrite(pinLight, LOW);
            lightState = true;
        } else {
            if ((millis() - lightTime) >= 500) {
                digitalWrite(pinLight, HIGH);
            }
            if ((millis() - lightTime) >= 1000) {
                lightState = false;
            }
        }

        if (buttonState == LOW) {
            if (calibrationSide) {
                if (!calibrateLeft) {
                    startTime = millis();
                    digitalWrite(pinLeftOpen, LOW);
                    calibrateLeft = true;
                } else if ((millis() - startTime) >= calibrationLimit) {
                    Serial.println("Calibration time limit reached!");
                    digitalWrite(pinLeftOpen, HIGH);
                    calibrateLeft = false;
                }
            } else {
                if (!calibrateRight) {
                    startTime = millis();
                    digitalWrite(pinRightOpen, LOW);
                    calibrateRight = true;
                } else if ((millis() - startTime) >= calibrationLimit) {
                    Serial.println("Calibration time limit reached!");
                    digitalWrite(pinRightOpen, HIGH);
                    calibrateRight = false;
                }
            }
        } else {
            if (calibrateLeft || calibrateRight) {
                digitalWrite(pinLeftOpen, HIGH);
                digitalWrite(pinRightOpen, HIGH);
                endTime = millis();
                if (calibrationSide) {
                    leftCalibrationTime += (endTime - startTime);
                } else {
                    rightCalibrationTime += (endTime - startTime);
                }
                calibrateLeft = false;
                calibrateRight = false;
            }
        }

        if (remotePulse) {
            if (calibrationSide) {
                Serial.println("Calibrating time to open/close left gate: " + String(leftCalibrationTime) + " ms");
                EEPROM.write(0, leftCalibrationTime / 100);
            } else {
                Serial.println("Calibrating time to open/close right gate: " + String(rightCalibrationTime) + " ms");
                EEPROM.write(1, rightCalibrationTime / 100);
            }
            calibrationSide = !calibrationSide;
        }
    }
    lastRemoteState = remoteState;
    lastLightswitchState = lightswitchState;
    lastKeypadState = keypadState;
    lastButtonState = buttonState;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}