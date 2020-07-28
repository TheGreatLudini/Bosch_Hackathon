#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const uint8_t INTERRUPT_PIN = 2;
const int numberOfIterations = 30;
const int maxButton = 10;
unsigned long lastInterrupt = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> xGlobal(1.0, 0.0, 0.0);
imu::Vector<3> yGlobal(0.0, 1.0, 0.0);
imu::Vector<3> zGlobal(0.0, 0.0, 1.0);

imu::Vector<3> xLocal;
imu::Vector<3> yLocal;
imu::Vector<3> zLocal;

double x1Arr[numberOfIterations * maxButton];
double x2Arr[numberOfIterations * maxButton];
double x3Arr[numberOfIterations * maxButton];
double y1Arr[numberOfIterations * maxButton];
double y2Arr[numberOfIterations * maxButton];
double y3Arr[numberOfIterations * maxButton];
double z1Arr[numberOfIterations * maxButton];
double z2Arr[numberOfIterations * maxButton];
double z3Arr[numberOfIterations * maxButton];

int buttonCounter = 0;
boolean interruptFlag = false;



void setup() {
    Serial.begin(9600);

    /* Initialise the sensor */
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //#ifndef DEBUG_BLE
    while(1);
    //#endif
    }

    bno.setExtCrystalUse(true);

    pinMode(INTERRUPT_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), storeAngles, FALLING);

    
}

void loop() {

    if (interruptFlag) {
        for (int j = 0; j < numberOfIterations; j++) {
            imu::Quaternion quat = bno.getQuat();
            imu::Vector<3> xLocal = quat.rotateVector(xGlobal);
            imu::Vector<3> yLocal = quat.rotateVector(yGlobal);
            imu::Vector<3> zLocal = quat.rotateVector(zGlobal);

            x1Arr[(buttonCounter - 1) * numberOfIterations + j] = xLocal.x();
            x2Arr[(buttonCounter - 1) * numberOfIterations + j] = xLocal.y();
            x3Arr[(buttonCounter - 1) * numberOfIterations + j] = xLocal.z();
            y1Arr[(buttonCounter - 1) * numberOfIterations + j] = yLocal.x();
            y2Arr[(buttonCounter - 1) * numberOfIterations + j] = yLocal.y();
            y3Arr[(buttonCounter - 1) * numberOfIterations + j] = yLocal.z();
            z1Arr[(buttonCounter - 1) * numberOfIterations + j] = zLocal.x();
            z2Arr[(buttonCounter - 1) * numberOfIterations + j] = zLocal.y();
            z3Arr[(buttonCounter - 1) * numberOfIterations + j] = zLocal.z();
        }
        Serial.println("ButtonCounter: " + buttonCounter);
        interruptFlag = false;

        if (buttonCounter == maxButton) {
            double x1Av = 0;
            double x2Av = 0;
            double x3Av = 0;
            double y1Av = 0;
            double y2Av = 0;
            double y3Av = 0;
            double z1Av = 0;
            double z2Av = 0;
            double z3Av = 0;
            for (int i = 0; i < maxButton * numberOfIterations; i++) {
                x1Av += x1Arr[i] / (maxButton * numberOfIterations);
                x2Av += x2Arr[i] / (maxButton * numberOfIterations); 
                x3Av += x3Arr[i] / (maxButton * numberOfIterations); 
                y1Av += y1Arr[i] / (maxButton * numberOfIterations); 
                y2Av += y2Arr[i] / (maxButton * numberOfIterations); 
                y3Av += y3Arr[i] / (maxButton * numberOfIterations); 
                z1Av += z1Arr[i] / (maxButton * numberOfIterations); 
                z2Av += z2Arr[i] / (maxButton * numberOfIterations); 
                z3Av += z3Arr[i] / (maxButton * numberOfIterations);  
            }
            Serial.print("x1Av: ");
            Serial.println(x1Av);
            Serial.print("x2Av: ");
            Serial.println(x2Av);
            Serial.print("x3Av: ");
            Serial.println(x3Av);
            Serial.print("y1Av: ");
            Serial.println(y1Av);
            Serial.print("y2Av: ");
            Serial.println(y2Av);
            Serial.print("y3Av: ");
            Serial.println(y3Av);
            Serial.print("z1Av: ");
            Serial.println(z1Av);
            Serial.print("z2Av: ");
            Serial.println(z2Av);
            Serial.print("z3Av: ");
            Serial.println(z3Av);
        }
    }
    

    
}

void storeAngles() {
    if (millis() - lastInterrupt > 60) {
        interruptFlag = true;
        buttonCounter++;
        lastInterrupt = millis();
    }
}