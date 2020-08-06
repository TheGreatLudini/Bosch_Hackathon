#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <ArduinoBLE.h>


#include "config.h"
    
//#define DEBUG_LOCAL_VECTOR
//#define DEBUG_MOTOR
//#define DEBUG_TOTAL_ERROR
//#define DEBUG_ERROR_DIR
#define DEBUG_CURRENT
#define DEBUG_ACCELERATION
//#define DEBUG_INTERRUPT
//#define DEBUG_ANGLE_DISPLACEMENT_MOTOR

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t GREEN = strip.Color(0, 255, 0);
const uint32_t RED = strip.Color(255, 0, 0);
const uint32_t BLUE = strip.Color(0, 0, 255);
const uint32_t ORANGE = strip.Color(255, 120, 0);
const uint32_t BLACK = strip.Color(0, 0, 0);

imu::Vector<3> drillDir(0.0, 0.0, 0.0);
imu::Vector<3> wallNormal(0.0, 0.0, 0.0);
imu::Vector<3> wallZ(0.0, 0.0, 0.0);
bool initialize = true;
bool preciceInitialize = false;
bool initGuard = false;
uint32_t interruptTime(0);
uint8_t interruptCounter(0);

uint32_t buttonPressDown(0);
bool drillAngleChanged = false;

sensors_event_t accelerationData;
double motorCurrentHistory[FILTERLENGTH]; 
double voltageHistory[VOLT_FILTERLENGTH]; 
uint32_t counter(0);
uint32_t voltCounter(0);
bool forwardDir = true;

bool motorStartUp = false;
bool motorOFF = false;

bool lastTriggerState = false; // false if trigger not pushed
bool currentTriggerState = false;
uint32_t triggerPushTime = 0;

double localLeftRightError;
double localUpDownError;

uint32_t lastSendTime = 0;
 
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);

    //init LEDs
    strip.begin();
    strip.setBrightness(20);
    strip.fill(BLUE, 0 , LED_COUNT);
    strip.show();

    Serial.println("Orientation Sensor Test"); 
    Serial.println("");

    
    /* Initialise the sensor */
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    #ifndef DEBUG_BLE
    while(1);
    #endif
    }
    
    delay(1000);
    Serial.println("Setup almost done");
    bno.setExtCrystalUse(true);
    Serial.println("Setup almost almost done");
    #ifdef BOARD_NANO
        pinMode(INTERRUPT_PIN, INPUT);
    #else
        pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    #endif

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TRIGGER_PIN, INPUT);
    pinMode(CURRENT_SENSE_PIN, INPUT);
    pinMode(MOTOR_SPEED_PIN, OUTPUT);
    pinMode(VOLT_FOR_PIN, INPUT);

    memset(motorCurrentHistory, 0, FILTERLENGTH * 8);
    Serial.println("Setup done");
}
    
void loop(void) 
{   
    

    

    lastTriggerState = currentTriggerState; 
    currentTriggerState = analogRead(TRIGGER_PIN) > VOLT_TRIGGER_TH;
    if ((currentTriggerState != lastTriggerState) && currentTriggerState) {
        #ifdef DEBUG_TRIGGER
        Serial.print("Trigger pushed: ");
        Serial.println(analogRead(TRIGGER_PIN));
        #endif
        triggerPushTime = millis();
    } else if ((currentTriggerState != lastTriggerState 
                && (millis() - triggerPushTime) < TRIGGER_MAX_DUR 
                && (millis() - triggerPushTime) > TRIGGER_MIN_DUR)) {
        setAngle();
    }
    
    double motorCurrent = CurrentMeasurment();
    double voltage = VoltageMeasurment();

    if (motorCurrent > CURRENT_TH) {
        double derrivative = getCurrentDerrivativ(motorCurrent);
        derrivative < 0 ? motorStartUp = true : true;
        if (motorStartUp && derrivative > DERR_CURRENT_TH) {
            motorOFF = true;
        }
    } else {
        motorStartUp = false;
    }
    !currentTriggerState ? motorOFF = false : true;

    bno.getEvent(&accelerationData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    
    //loopBLE(); 
    // -------------------
    // handel Initializations if those are set in the set angle interrupt
    if (!initGuard) {
        if (preciceInitialize) {
            preciceInit();
            initSequLed(BLUE);

        } else if (initialize) {
            Init();
            initSequLed(ORANGE);
        }
    } else if (abs(millis() - interruptTime) > 500) {
        initGuard = false;
    }
    // -------------------
    // calculate misalignment/errors
    imu::Quaternion quat = bno.getQuat(); // get oriantation 

    // Coordinate system axes of screw driver in global coordinates, x is drilling axis:
    imu::Vector<3> xLocal = quat.rotateVector(xGlobal);
    imu::Vector<3> yLocal = quat.rotateVector(yGlobal);
    imu::Vector<3> zLocal = quat.rotateVector(zGlobal);

    // Angle error in up-down- and left-right-direction, determined via the dot product
    // If the dot product is 0, the respective axis is orthogonal to the wall normal, therefore good
    localLeftRightError = drillDir.dot(zLocal);
    localUpDownError = drillDir.dot(yLocal);
    double localErrorTotal = sqrt(pow(localUpDownError, 2) + pow(localLeftRightError, 2));

    // -------------------
    // set Leds and Motorspeed acording to misalignment
    // calculate motorspeed
    uint8_t motorSpeed(0);
    if (localErrorTotal < MOTOR_ON_THESHOLD) {
        // The higher this motor speed variable, the slower the motor unfortunately
        motorSpeed = min(localErrorTotal * MOTOR_SPEED_SENSITIVITY, 255); 
        #ifdef DEBUG_ANGLE_DISPLACEMENT_MOTOR
        Serial.print("Good ");
        Serial.println(motorSpeed);
        #endif
    } else {
        // turn the motor off
        #ifdef DEBUG_ANGLE_DISPLACEMENT_MOTOR
        Serial.println("Bad");
        #endif
        motorSpeed = 255;
    }
    // set motorspeed motor speed of 255 means the motor is turned off
    // analogWrite(MOTOR_SPEED_PIN, motorSpeed);
    
    digitalWrite(MOTOR_SPEED_PIN, !motorOFF);

    // set leds
    setLeds(localLeftRightError, localUpDownError, localErrorTotal);
    
    // -------------------
    // Debug outputs
    #ifdef DEBUG_MOTOR
    Serial.print("Motor Speed: ");
    Serial.print(motorSpeed);
    #endif

    #ifdef DEBUG_ERROR_DIR
    Serial.print("LR_Error: ");
    Serial.print(localLeftRightError);
    Serial.print("\tUD_Error: ");
    Serial.print(localUpDownError);
    #endif

    #ifdef DEBUG_LOCAL_VECTOR
    // Serial.print("Alpha: ");
    // Serial.print(quat.toEuler().x() / 3.1416 * 180);
    // Serial.print("\tBeta: ");
    // Serial.print(quat.toEuler().y() / 3.1416 * 180);
    // Serial.print("\tGamma: ");
    // Serial.print(quat.toEuler().z() / 3.1416 * 180);
    Serial.print("\tX: ");
    Serial.print(xLocal.x());
    Serial.print("\tY: ");
    Serial.print(xLocal.y());
    Serial.print("\tZ: ");
    Serial.print(xLocal.z());
    #endif

    #ifdef DEBUG_ACCELERATION
    Serial.print("\taccX:");
    Serial.print(accelerationData.acceleration.x);
    #endif
    Serial.println();
}

/**
 * Handels the trigger push events and sets the corresponing 
 * initialization flags (preciceInitialize and initialize)
 */ 
void setAngle()
{
    #ifdef DEBUG_INTERRUPT
    Serial.println("Interrupt was made!");
    #endif
    if (abs(millis() - interruptTime) > 500) {
        interruptCounter = 0;
    }
    if (interruptCounter == 0) {
        interruptCounter++;
        preciceInitialize = true;
        initGuard = true;
        interruptTime = millis();
    } else if (abs(millis() - interruptTime) > debounce) {
        interruptCounter = 0;
        preciceInitialize = false;
        initialize = true;
        interruptTime = millis();
    }  
}

/**
 * Measures the motor current with a moving averafe of FILTERLENGTH
 * the raw data is stored in motorCurrentHistory wich is used for the moving average
 */
double CurrentMeasurment() {
    double motorCurrent(0);
    motorCurrentHistory[counter % FILTERLENGTH] = CURRENT_FACTOR * analogRead(CURRENT_SENSE_PIN);
    for (int i = 0; i < FILTERLENGTH; i++) {
        motorCurrent += motorCurrentHistory[i];
    }
    motorCurrent = motorCurrent / FILTERLENGTH;
    counter++;
    #ifdef DEBUG_CURRENT
        Serial.print("\tI :");
        Serial.print(motorCurrent);
    #endif
    return motorCurrent;
}
double getCurrentDerrivativ(double motorCurrent) {
    double derrivative(0);
    for (int i = counter % FILTERLENGTH; i > (counter % FILTERLENGTH) - DERRIV_LENGTH; i--) {
        derrivative += motorCurrent - motorCurrentHistory[i % FILTERLENGTH];
    }
    derrivative /= DERRIV_LENGTH;
    #ifdef DEBUG_CURRENT
        Serial.print("\tdI :");
        Serial.print(derrivative);
    #endif
    return derrivative;
}

/**
 * Measures the motor voltage with a moving average of VOLT_FILTERLENGTH 
 * the raw data is stored in motorCurrentHistory wich is used for the moving average
 */ 
double VoltageMeasurment() {
    double voltage(0);
    uint16_t voltForw = analogRead(VOLT_FOR_PIN); 
    //uint16_t voltBack = analogRead(VOLT_BACK_PIN);

    forwardDir = true;
    voltageHistory[voltCounter % VOLT_FILTERLENGTH] = VOLTAGE_FACTOR * (voltForw);
   
    for (int i = 0; i < VOLT_FILTERLENGTH; i++) {
        voltage += voltageHistory[i];
    }
    voltage /= VOLT_FILTERLENGTH;
    voltCounter++;
    #ifdef DEBUG_VOLTAGE
        Serial.print("\tU :");
        Serial.print(voltage);
    #endif
    return voltage;

}

/**
 * show the Led sequence when the inizialisation was sucessful 
 * 
 * @param color the color in wich the leds are to be shown 
 */
void initSequLed(uint32_t color) {
    strip.clear();
    strip.show();
    for(int i = 1; i <= LED_COUNT - 1; i++) {
        i % 3 ? strip.setPixelColor(LedTop, BLACK) : strip.setPixelColor(LedTop, color);
        strip.fill(color, 1, i);
        strip.show();
        delay(50);
    }
    strip.clear();
    strip.setPixelColor(LedCenter, color);
    strip.setPixelColor(LedTop, color);
    strip.show();
    delay(500);
}

void setLeds(double localLeftRightError, double localUpDownError, double localErrorTotal) {
    if (localUpDownError >= 0) {
        strip.setPixelColor(LedUp, 0,  LED_INCREASE * localUpDownError * 255, 0);
        strip.setPixelColor(LedDown, BLACK);
    } else {
        strip.setPixelColor(LedDown, 0, - LED_INCREASE * localUpDownError * 255, 0);
        strip.setPixelColor(LedUp, BLACK);
    }
    if (localLeftRightError <= 0) {
        strip.setPixelColor(LedLeft, 0, - LED_INCREASE * localLeftRightError * 255, 0);
        strip.setPixelColor(LedRight, BLACK);
    } else {
        strip.setPixelColor(LedRight, 0,  LED_INCREASE * localLeftRightError * 255, 0);
        strip.setPixelColor(LedLeft, BLACK);
    }
    if (localErrorTotal < CENTER_LED_ON_THESHOLD) {
        strip.setPixelColor(LedCenter, 0, (1.0 - CENTER_LED_DECREASE * localErrorTotal) * 255, 0);
    } else {
        strip.setPixelColor(LedCenter, BLACK);
    }
    strip.show();
}

/**
 * initializes the Wall coordinates assuming the drill
 * is hold against a wall
 */
void preciceInit() {
    preciceInitialize = false;
    imu::Quaternion quat = bno.getQuat();
    wallNormal = quat.rotateVector(yGlobal);
    wallZ = quat.rotateVector(zGlobal);
    wallNormal = RotDir(ANGLE_DISPLACEMENT, wallZ, wallNormal);
    Serial.print("X: ");
    Serial.print(wallNormal.x());
    Serial.print("\tY: ");
    Serial.print(wallNormal.y());
    Serial.print("\tZ: ");
    Serial.println(wallNormal.z());
    drillDir = wallNormal;
}

/**
 * initializes the Wall coordinates assuming the drill
 * faces the direction the user wants to drill
 */
void Init() {
    initialize = false;
    imu::Quaternion quat = bno.getQuat();
    wallZ = quat.rotateVector(zGlobal).invert();
    wallNormal = quat.rotateVector(xGlobal);
    drillDir = wallNormal;
    Serial.print("X: ");
    Serial.print(wallNormal.x());
    Serial.print("\tY: ");
    Serial.print(wallNormal.y());
    Serial.print("\tZ: ");
    Serial.println(wallNormal.z());
}

/**
 * Rotates the Drilling direction in relation to the Wall coordinates
 * 
 * @param angle Rotagion angle in degree
 * @param rotAxis the rotation axis of the rotation
 */
imu::Vector<3> RotDir(double angle, imu::Vector<3> rotAxis, imu::Vector<3> VecToRotate) {
    double phi = angle / 180 * 3.1416;
    imu::Quaternion rotQuat(cos(phi / 2), rotAxis.scale(sin(phi / 2)));
    return rotQuat.rotateVector(VecToRotate);
}