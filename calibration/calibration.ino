#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const uint8_t INTERRUPT_PIN = 2;
const int numberOfIterations = 100;
const int maxButton = 10;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> xGlobal(1.0, 0.0, 0.0);
imu::Vector<3> yGlobal(0.0, 1.0, 0.0);
imu::Vector<3> zGlobal(0.0, 0.0, 1.0);

imu::Vector<3> xLocal;
imu::Vector<3> yLocal;
imu::Vector<3> zLocal;

double x1[numberOfIterations * maxButton];
double x2[numberOfIterations * maxButton];
double x3[numberOfIterations * maxButton];
double y1[numberOfIterations * maxButton];
double y2[numberOfIterations * maxButton];
double y3[numberOfIterations * maxButton];
double z1[numberOfIterations * maxButton];
double z2[numberOfIterations * maxButton];
double z3[numberOfIterations * maxButton];

int buttonCounter = 0;
boolean interruptFlag = false;



void setup() {
    Serial.begin(9600);

    /* Initialise the sensor */
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    #ifndef DEBUG_BLE
    while(1);
    #endif
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

            x1[(buttonCounter - 1) * numberOfIterations + j] = xLocal[0];
            x2[(buttonCounter - 1) * numberOfIterations + j] = xLocal[1];
            x3[(buttonCounter - 1) * numberOfIterations + j] = xLocal[2];
            y1[(buttonCounter - 1) * numberOfIterations + j] = yLocal[0];
            y2[(buttonCounter - 1) * numberOfIterations + j] = yLocal[1];
            y3[(buttonCounter - 1) * numberOfIterations + j] = yLocal[2];
            z1[(buttonCounter - 1) * numberOfIterations + j] = zLocal[0];
            z2[(buttonCounter - 1) * numberOfIterations + j] = zLocal[1];
            z3[(buttonCounter - 1) * numberOfIterations + j] = zLocal[2];
        }
        Serial.println("ButtonCounter: " + buttonCounter);
        interruptFlag = false;

        if (buttonCounter == maxButton) {
            int x1Av = 0;
            int x2Av = 0;
            int x3Av = 0;
            int y1Av = 0;
            int y2Av = 0;
            int y3Av = 0;
            int z1Av = 0;
            int z2Av = 0;
            int z3Av = 0;
            for (int i = 0; i < maxButton * numberOfIterations; i++) {
                x1Av += x1[i] / maxButton * numberOfIterations;
                x2Av += x2[i] / maxButton * numberOfIterations; 
                x3Av += x3[i] / maxButton * numberOfIterations; 
                y1Av += y1[i] / maxButton * numberOfIterations; 
                y2Av += y2[i] / maxButton * numberOfIterations; 
                y3Av += y3[i] / maxButton * numberOfIterations; 
                z1Av += z1[i] / maxButton * numberOfIterations; 
                z2Av += z2[i] / maxButton * numberOfIterations; 
                z3Av += z3[i] / maxButton * numberOfIterations;  
            }
            Serial.println("x1Av: " + x1Av);
            Serial.println("x2Av: " + x2Av);
            Serial.println("x3Av: " + x3Av);
            Serial.println("y1Av: " + y1Av);
            Serial.println("y2Av: " + y2Av);
            Serial.println("y3Av: " + y3Av);
            Serial.println("z1Av: " + z1Av);
            Serial.println("z2Av: " + z2Av);
            Serial.println("z3Av: " + z3Av);
        }
    }
    

    
}

void storeAngles() {
    interruptFlag = true;
    buttonCounter++;
}