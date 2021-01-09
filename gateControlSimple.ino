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

bool remoteState = false; 
bool oldRemoteState = false; 
bool lightswitchState = false; 
bool oldLightswitchState = false; 
bool keypadState = false;
bool oldKeypadState = false;

bool gateOpening = false;
bool gateClosing = false;
bool gateState = false;

unsigned long openLeftTime = 0;
unsigned long openRightTime = 0;
unsigned long closeLeftTime = 0;
unsigned long closeRightTime = 0;
unsigned long lightTime = 0;

bool lightState = false;

bool openLeftStarted = false;
bool openRightStarted = false;

bool closeLeftStarted = false;
bool closeRightStarted = false;


int timeRuntime = 12000;


void setup() {
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
}

void loop() {
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
        } else {
            if ((millis() - lightTime) >= 1000) {
                digitalWrite(pinLight, HIGH);
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
                if ((millis() - openLeftTime) >= timeRuntime) {
                    Serial.println("Opening Left Gate - Stop");
                    digitalWrite(pinLeftOpen, HIGH);
                    delay(500);
                    Serial.println("Opening Right Gate - Start");
                    digitalWrite(pinRightOpen, LOW);
                    openRightTime = millis();
                    openRightStarted = true;
                }
            } else {
                if ((millis() - openRightTime) >= timeRuntime) {
                    Serial.println("Opening Right Gate - Stop");
                    digitalWrite(pinRightOpen, HIGH);
                    digitalWrite(pinLight, HIGH);
                    openLeftStarted = false;
                    openRightStarted = false;
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
        } else {
            if ((millis() - lightTime) >= 1000) {
                digitalWrite(pinLight, HIGH);
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
            if (!closeLeftStarted) {
                if ((millis() - closeRightTime) >= timeRuntime) {
                    Serial.println("Closing Right Gate - Stop");
                    digitalWrite(pinRightClose, HIGH);
                    delay(500);
                    Serial.println("Closing Left Gate - Start");
                    digitalWrite(pinLeftClose, LOW);
                    closeLeftTime = millis();
                    closeLeftStarted = true;
                }
            } else {
                if ((millis() - closeLeftTime) >= timeRuntime) {
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
            }
        }
    }
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}