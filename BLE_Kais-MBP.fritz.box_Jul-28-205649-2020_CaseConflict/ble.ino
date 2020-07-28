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
//#define DEBUG_CURRENT
//#define DEBUG_ACCELERATION
//#define DEBUG_INTERRUPT

//#define DEBUG_BLE
//#define DEBUG_BLE_RECIVE


Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t GREEN = strip.Color(0, 255, 0);
const uint32_t RED = strip.Color(255, 0, 0);
const uint32_t BLACK = strip.Color(0, 0, 0);

imu::Vector<3> drillDir(0.0, 0.0, 0.0);
imu::Vector<3> wallNormal(0.0, 0.0, 0.0);
imu::Vector<3> wall_X(0.0, 0.0, 0.0);
imu::Vector<3> wall_Y(0.0, 0.0, 0.0);
imu::Vector<3> wall_Z(0.0, 0.0, 0.0);
bool initialize = true;
bool preciceInitialize = false;
bool initGuard = false;
uint32_t interruptTime(0);
uint8_t interruptCounter(0);

sensors_event_t accelerationData;
double motorCurrentHistory[FILTERLENGTH]; 
double voltageHistory[VOLT_FILTERLENGTH]; 
uint32_t counter(0);
uint32_t voltCounter(0);
bool forwardDir = true;
bool buttonStateOld = false; // false is high
bool buttonState = false;

double localLeftRightError;
double localUpDownError;

uint32_t lastSendTime = 0;

//Motor myMotor = new Motor(MOTOR_FOR_DIR_PIN, MOTOR_BACK_DIR_PIN, MOTOR_SPEED_PIN);
    
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);

    //init LEDs
    strip.begin();
    strip.setBrightness(20);
    strip.fill(RED, 0 , LED_COUNT);
    strip.show();
    strip.clear();
    strip.show();
    for(int i = 1; i <= LED_COUNT; i++) {
        strip.fill(RED, 0, i);
	    strip.show();
        delay(100);
    }
    strip.clear();
    strip.setPixelColor(LedCenter, RED);
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
    pinMode(CURRENT_SENSE_PIN, INPUT);
    pinMode(MOTOR_SPEED_PIN, OUTPUT);
    pinMode(VOLT_BACK_PIN, INPUT);
    pinMode(VOLT_FOR_PIN, INPUT);

    memset(motorCurrentHistory, 0, FILTERLENGTH * 8);
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setAngle, LOW);
    //initBLE();
    Serial.println("Setup done");
}
    
void loop(void) 
{   
    buttonStateOld = buttonState;
    buttonState = (digitalRead(INTERRUPT_PIN) == LOW);
    if ((buttonState != buttonStateOld) && !buttonState) {
        setAngle();
    }

    //if (digitalRead(INTERRUPT_PIN) == LOW) {
        //setAngle();
    //}
    double motorCurrent = CurrentMeasurment();
    double voltage = VoltageMeasurment();
    bno.getEvent(&accelerationData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    
    //loopBLE(); 
    //-------------------
    // Triggerdetection
    if(voltage > TRIGGER_DEBOUNCE) {
   
    }
    // -------------------
    // handel Initializations if those are set in the set angle interrupt
    if (!initGuard) {
        if (preciceInitialize) {
            preciceInit();
        } else if (initialize) {
            Init();
        }
    } else if (abs(millis() - interruptTime) > 500) {
        initGuard = false;
    }
    // -------------------
    // calculate misalignment/errors
    imu::Quaternion quat = bno.getQuat();
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
    uint8_t motorSpeed(0);
    if (localErrorTotal < MOTOR_ON_THESHOLD) {
        Serial.print("Good ");
        // The higher this motor speed variable, the slower the motor unfortunately
        motorSpeed = min(localErrorTotal * MOTOR_SPEED_SENSITIVITY, 255); 
        Serial.println(motorSpeed);
    } else {
        Serial.println("Bad");
        motorSpeed = 255;
    }
    analogWrite(MOTOR_SPEED_PIN, motorSpeed);
    // analogWrite(MOTOR_SPEED_PIN, 255 - motorSpeed);

    setLeds(localLeftRightError, localUpDownError, localErrorTotal);
    
    // -------------------
    // Debug outputs
    #ifdef DEBUG_MOTOR
    Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);
    #endif

    #ifdef DEBUG_ERROR_DIR
    Serial.print("LR_Error: ");
    Serial.print(localLeftRightError);
    Serial.print("\tUD_Error: ");
    Serial.println(localUpDownError);
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
    Serial.println(xLocal.z());
    #endif

    #ifdef DEBUG_ACCELERATION
    Serial.print("\taccX: ");
    Serial.println(accelerationData.acceleration.x);
    #endif

    //delay(100);
}

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
        //Serial.println("Initializing");    
}

double CurrentMeasurment() {
    double motorCurrent(0);
    motorCurrentHistory[counter % FILTERLENGTH] = CURRENT_FACTOR * analogRead(CURRENT_SENSE_PIN);
    for (int i = 0; i < FILTERLENGTH; i++) {
        motorCurrent += motorCurrentHistory[i];
    }
    motorCurrent = motorCurrent / FILTERLENGTH;
    counter++;
    #ifdef DEBUG_CURRENT
        Serial.print("I :");
        Serial.print(motorCurrent);
    #endif
    return motorCurrent;

}

double VoltageMeasurment() {
    double voltage(0);
    uint16_t voltForw = analogRead(VOLT_FOR_PIN); 
    uint16_t voltBack = analogRead(VOLT_BACK_PIN);
    if (voltForw > voltBack) {
        forwardDir = true;
        voltageHistory[voltCounter % VOLT_FILTERLENGTH] = VOLTAGE_FACTOR * (voltForw - voltBack);
    } else {
        forwardDir = false;
        voltageHistory[voltCounter % VOLT_FILTERLENGTH] = VOLTAGE_FACTOR * (voltBack - voltForw);
    }    
    for (int i = 0; i < VOLT_FILTERLENGTH; i++) {
        voltage += voltageHistory[i];
    }
    voltage /= VOLT_FILTERLENGTH;
    voltCounter++;
    #ifdef DEBUG_CURRENT
        Serial.print("\tU :");
        Serial.println(voltage);
    #endif
    return voltage;

}

void setLeds(double localLeftRightError, double localUpDownError, double localErrorTotal) {
    if (localUpDownError >= 0) {
        strip.setPixelColor(LedUp, 0, localUpDownError * 255, 0);
        strip.setPixelColor(LedDown, BLACK);
    } else {
        strip.setPixelColor(LedDown, 0, -localUpDownError * 255, 0);
        strip.setPixelColor(LedUp, BLACK);
    }
    if (localLeftRightError <= 0) {
        strip.setPixelColor(LedLeft, 0, localLeftRightError * 255, 0);
        strip.setPixelColor(LedRight, BLACK);
    } else {
        strip.setPixelColor(LedRight, 0, -localLeftRightError * 255, 0);
        strip.setPixelColor(LedLeft, BLACK);
    }
    if (localErrorTotal < CENTER_LED_ON_THESHOLD) {
        strip.setPixelColor(LedCenter, 0, (CENTER_LED_ON_THESHOLD - localErrorTotal) * 255, 0);
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
    wallNormal = RotDir(ANGLE_DISPLACEMENT, quat.rotateVector(zGlobal).invert(), wallNormal);
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


// void initBLE(){

//     // begin initialization
//     if (!BLE.begin()) {
//         Serial.println("starting BLE failed!");
//         while (1);
//     }

//     /* Set a local name for the BLE device
//         This name will appear in advertising packets
//         and can be used by remote devices to identify this BLE device
//         The name can be changed but maybe be truncated based on space left in advertisement packet
//     */
//     BLE.setEventHandler(BLEConnected, ConnectHandler);
//     BLE.setEventHandler(BLEDisconnected, DisconnectHandler);
//     BLE.setLocalName("Schraubenmaster4000");
//     Serial.println("BLE name set");
//     Serial.println(BLE.address());


//     //BLE.setAdvertisedService(angleService); // add the service UUID

//     angleService.addCharacteristic(SetAngleCharLR); // add the battery level characteristic
//     angleService.addCharacteristic(SetAngleCharUD); // add the battery level characteristic
//     angleService.addCharacteristic(SendScalarUD); // add the battery level characteristic
//     angleService.addCharacteristic(SendScalarLR); // add the battery level characteristic
//     angleService.addCharacteristic(CalibrateChar); // add the battery level characteristic


//     BLE.addService(angleService);

//     SetAngleCharLR.writeValue(1111111111); // set initial value for this characteristic
//     SetAngleCharUD.writeValue(1111111111); // set initial value for this characteristic
//     SendScalarLR.writeValue(1111111111); // set initial value for this characteristic
//     SendScalarUD.writeValue(1111111111); // set initial value for this characteristic
//     CalibrateChar.writeValue(0); // set initial value for this characteristic


//     // start advertising
//     BLE.advertise();

//     Serial.println("Bluetooth device active, waiting for connections...");
    
// }

// void loopBLE(){
//     // wait for a BLE central
//     BLEDevice central = BLE.central();
    
    
//     // if a central is connected to the peripheral:
//     if (central) {
        
//         // #ifdef DEBUG_BLE
//         // Serial.print("Connected to central: ");
//         // // print the central's BT address:
//         // Serial.println(central.address());
//         // #endif
//         if(CalibrateChar.written()) {
//             preciceInit();
//         }
//         double LR(0);
//         double UD(0);
//         bool newDrillAngle = false;
//         if(SetAngleCharLR.written()){
//             newDrillAngle = true;
//             double LR = SetAngleCharLR.value(); 
//             #ifdef DEBUG_BLE_RECIVE
//             Serial.print("recived LR: ");
//             Serial.println(LR);
//             #endif
//         }
//         if(SetAngleCharUD.written()) {
//             newDrillAngle = true;
//             double UD = SetAngleCharLR.value(); 
//             #ifdef DEBUG_BLE_RECIVE
//             Serial.print("recived UD: ");
//             Serial.println(UD);
//             #endif
//         }
//         if (newDrillAngle) {
//             drillDir = wallNormal;
//             drillDir = RotDir(LR, wall_X, drillDir);
//             drillDir = RotDir(UD, wall_Y, drillDir);
//         }
//         if (abs(millis() - lastSendTime) > 500) {
//             SendScalarUD.writeValue((float)localUpDownError);
//             // Serial.println("I wrote a value to BLE char.");
//             // SendScalarUD.valueUpdated();
//             SendScalarLR.writeValue((float)localLeftRightError);
//             lastSendTime = millis();
//         }
//         #ifdef DEBUG_BLE
//             Serial.print("Send UD: ");
//             Serial.print((float)localUpDownError);
//             Serial.print("\tSend LR: ");
//             Serial.println((float)localUpDownError);
//         #endif

//     }
//     if(CalibrateChar.written()){
//         CalibrateChar.writeValue(0);
//     }
// }

// /**
//  * Encodes Values that are send over BLE 
//  * 2 values with 5 digits each 
//  * first digit: 1 -> positve value 
//  *              0 -> negative Value
//  */
// unsigned int encodeValue(double leftRight, double upDown) {
//     uint32_t LR = abs(leftRight) * 1000;
//     uint32_t UD = abs(upDown) * 1000;
//     leftRight > 0 ? LR += 10000 : LR = LR;
//     upDown > 0 ? UD += 10000 : UD = UD;
//     uint32_t retVal = (LR * 100000) + UD;

//     #ifdef DEBUG_BLE
//         Serial.print("LR_Error: ");
//         Serial.print(localLeftRightError);
//         Serial.print("\tUD_Error: ");
//         Serial.println(localUpDownError);
//         Serial.println(retVal);
//     #endif
//     return retVal;
// }
