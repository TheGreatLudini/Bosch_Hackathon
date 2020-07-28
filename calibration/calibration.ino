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
        if (buttonCounter <= maxButton) {
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
            Serial.print("ButtonCounter: ");
            Serial.println(buttonCounter);
            interruptFlag = false;
        }

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
                x1Av += x1Arr[i];
                x2Av += x2Arr[i]; 
                x3Av += x3Arr[i]; 
                y1Av += y1Arr[i]; 
                y2Av += y2Arr[i]; 
                y3Av += y3Arr[i]; 
                z1Av += z1Arr[i]; 
                z2Av += z2Arr[i]; 
                z3Av += z3Arr[i]; 
                Serial.println(String(y2Av, '\006')); 
            }
            x1Av /= (maxButton * numberOfIterations);
            x2Av /= (maxButton * numberOfIterations); 
            x3Av /= (maxButton * numberOfIterations); 
            y1Av /= (maxButton * numberOfIterations); 
            y2Av /= (maxButton * numberOfIterations); 
            y3Av /= (maxButton * numberOfIterations); 
            z1Av /= (maxButton * numberOfIterations); 
            z2Av /= (maxButton * numberOfIterations); 
            z3Av /= (maxButton * numberOfIterations); 
            Serial.print("x1Av: ");
            Serial.println(String(x1Arr[0], '\005'));
            Serial.print("x2Av: ");
            Serial.println(String(x2Arr[0], '\005'));
            Serial.print("x3Av: ");
            Serial.println(String(x3Arr[0], '\005'));
            Serial.print("y1Av: ");
            Serial.println(String(y1Arr[0], '\005'));
            Serial.print("y2Av: ");
            Serial.println(String(y2Arr[0], '\005'));
            Serial.print("y3Av: ");
            Serial.println(String(y3Arr[0], '\005'));
            Serial.print("z1Av: ");
            Serial.println(String(z1Arr[0], '\005'));
            Serial.print("z2Av: ");
            Serial.println(String(z2Arr[0], '\005'));
            Serial.print("z3Av: ");
            Serial.println(String(z3Arr[0], '\005'));
            Serial.print("x1Av: ");
            Serial.println(String(x1Av, '\005'));
            Serial.print("x2Av: ");
            Serial.println(String(x2Av, '\005'));
            Serial.print("x3Av: ");
            Serial.println(String(x3Av, '\005'));
            Serial.print("y1Av: ");
            Serial.println(String(y1Av, '\005'));
            Serial.print("y2Av: ");
            Serial.println(String(y2Av, '\005'));
            Serial.print("y3Av: ");
            Serial.println(String(y3Av, '\005'));
            Serial.print("z1Av: ");
            Serial.println(String(z1Av, '\005'));
            Serial.print("z2Av: ");
            Serial.println(String(z2Av, '\005'));
            Serial.print("z3Av: ");
            Serial.println(String(z3Av, '\005'));
            delay(1000000);
        }
    }
    

    
}

void storeAngles() {
    if (millis() - lastInterrupt > 1000) {
        interruptFlag = true;
        buttonCounter++;
        lastInterrupt = millis();
    }
}