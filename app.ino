#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "config.h"
// #include "Motor.h"
    
// #define DEBUG_LOCAL_VECTOR
#define DEBUG_ERROR_DIR
//#define DEBUG_MISSALIGN


Adafruit_BNO055 bno = Adafruit_BNO055(55);

imu::Vector<3> wallNormal(0.0, 0.0, 0.0);
imu::Vector<3> wall_X(0.0, 0.0, 0.0);
imu::Vector<3> wall_Y(0.0, 0.0, 0.0);
bool initialize = true;
bool preciceInitialize = false;
bool initGuard = false;
uint32_t interruptTime(0);
uint8_t interruptCounter(0);

//Motor myMotor = new Motor(MOTOR_FOR_DIR_PIN, MOTOR_BACK_DIR_PIN, MOTOR_SPEED_PIN);
    
void setup(void) 
{
    Serial.begin(9600);
    Serial.println("Orientation Sensor Test"); Serial.println("");
    
    /* Initialise the sensor */
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
    }
    
    delay(1000);
    Serial.println("Setup almost done");
    bno.setExtCrystalUse(true);
    Serial.println("Setup almost almost done");
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_UP, OUTPUT);
    pinMode(LED_RIGHT, OUTPUT);
    pinMode(LED_DOWN, OUTPUT);
    pinMode(LED_LEFT, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setAngle, FALLING);
    Serial.println("Setup done");
}
    
void loop(void) 
{   
    // The current orientation of the screw driver is stored as a quaternion in "quat":
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> vectorToRotate(1.0, 0, 0);
    imu::Vector<3> rotatedVector = quat.rotateVector(vectorToRotate);

    if (!initGuard) {
        if (preciceInitialize) {
            preciceInit();
        } else if (initialize) {
            Init();
        }
    } else if (abs(millis() - interruptTime) > 2 * debounce) {
        initGuard = false;
    }

    imu::Vector<3> xGlobal(1.0, 0.0, 0.0);
    imu::Vector<3> yGlobal(0.0, 1.0, 0.0);
    imu::Vector<3> zGlobal(0.0, 0.0, 1.0);
    // Coordinate system axes of screw driver in global coordinates, x is drilling axis:
    imu::Vector<3> xLocal = quat.rotateVector(xGlobal);
    imu::Vector<3> yLocal = quat.rotateVector(yGlobal);
    imu::Vector<3> zLocal = quat.rotateVector(zGlobal);

    // Angle error in up-down- and left-right-direction, determined via the dot product
    // If the dot product is 0, the respective axis is orthogonal to the wall normal, therefore good
    double localLeftRightError = wallNormal.dot(yLocal);
    double localUpDownError = wallNormal.dot(zLocal);

    // LED on if the drilling angle is correct
    digitalWrite(LED_BUILTIN, abs(localLeftRightError) < ANGLE_DISPLACEMENT && abs(localUpDownError) < ANGLE_DISPLACEMENT);

    
    if (localUpDownError >= 0) {
        analogWrite(LED_UP, localUpDownError * 255);
        analogWrite(LED_DOWN, 0);
    } else {
        analogWrite(LED_DOWN, -(localUpDownError * 255));
        analogWrite(LED_UP, 0);
    }
    if (localLeftRightError >= 0) {
        analogWrite(LED_LEFT, localLeftRightError * 255);
        analogWrite(LED_RIGHT, 0);
    } else {
        analogWrite(LED_RIGHT, -(localLeftRightError * 255));
        analogWrite(LED_LEFT, 0);
    }
    

    #ifdef DEBUG_ERROR_DIR
    Serial.print("LR_Error: ");
    Serial.print(localLeftRightError);
    Serial.print("\tUD_Error: ");
    Serial.println(localUpDownError);
    #endif

    #ifdef DEBUG_MISSALIGN
    Serial.print("Missaligment Left: ");
    Serial.print(localLeftRightError);
    Serial.print("\tDOWN: ");
    Serial.println(localUpDownError);
    #endif

    #ifdef DEBUG_LOCAL_VECTOR
    Serial.print("Alpha: ");
    Serial.print(quat.toEuler().x() / 3.1416 * 180);
    Serial.print("\tBeta: ");
    Serial.print(quat.toEuler().y() / 3.1416 * 180);
    Serial.print("\tGamma: ");
    Serial.println(quat.toEuler().z() / 3.1416 * 180);
    Serial.print("X: ");
    Serial.print(xLocal.x());
    Serial.print("\tY: ");
    Serial.print(yLocal.y());
    Serial.print("\tZ: ");
    Serial.println(zLocal.z());
    #endif
    delay(100);


}

void setAngle()
{
    if (interruptCounter == 0 && abs(millis() - interruptTime) > 1000) {
        interruptCounter++;
        preciceInitialize = true;
        initGuard = true;
    } else if (abs(millis() - interruptTime) > debounce) {
        interruptCounter = 0;
        initialize = true;
    }

    interruptTime = millis();
    Serial.println("Initializing");
}

/**
 * initializes the Wall coordinates assuming the drill
 * is hold against a wall
 */
void preciceInitialize() {
    preciceInitialize = false;
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> Xvector(1.0, 0, 0);
    imu::Vector<3> Yvector(0, 1.0, 0);
    imu::Vector<3> Zvector(0, 0, 1.0);
    wallNormal = quat.rotateVector(Zvector);
    wall_Y = quat.rotateVector(Yvector);
    wall_X = quat.rotateVector(Xvector);
    double phi = DRILL_ANGLE_OFFSET / 180 * 3.1416;
    imu::Quaternion rotQuat(cos(phi / 2), wall_Y.scale(sin(phi / 2)));
    wallNormal = rotQuat.rotateVector(wallNormal);
    Serial.print("X: ");
    Serial.print(wallNormal.x());
    Serial.print("\tY: ");
    Serial.print(wallNormal.y());
    Serial.print("\tZ: ");
    Serial.println(wallNormal.z());
}

/**
 * initializes the Wall coordinates assuming the drill
 * faces the direction the user wants to drill
 */
void Init() {
    initialize = false;
    preciceInitialize = false;
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> Xvector(1.0, 0, 0);
    imu::Vector<3> Yvector(0, 1.0, 0);
    imu::Vector<3> Zvector(0, 0, 1.0);
    wallNormal = quat.rotateVector(Xvector);
    wall_Y = quat.rotateVector(Zvector);
    wall_X = quat.rotateVector(Yvector);
    Serial.print("X: ");
    Serial.print(wallNormal.x());
    Serial.print("\tY: ");
    Serial.print(wallNormal.y());
    Serial.print("\tZ: ");
    Serial.println(wallNormal.z());
}

void getCartesian(double* angles, double* vecToRotate, double* cartesian) {
    double rotMat[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    getRot(angles, (double*)rotMat);
    matrixMultip((double*)rotMat, vecToRotate, cartesian, 3, 3, 3, 1);
}


/**
 * conversion of euler values to cartesian with XYZ cnvention
 */
void getRot(double* angles, double* rotMat) {
    rotMat[0] = cos(angles[1]) * cos(angles[2]);
    rotMat[1] = cos(angles[0]) * sin(angles[2]) + cos(angles[2]) * sin(angles[0]) * sin(angles[1]);
    rotMat[2] = sin(angles[0]) * sin(angles[2]) - cos(angles[0]) * cos(angles[2]) * sin(angles[1]);
    rotMat[3] = -cos(angles[1]) * sin(angles[2]);
    rotMat[4] = cos(angles[0]) * cos(angles[2]) - sin(angles[0]) * sin(angles[1]) * sin(angles[2]);
    rotMat[5] = sin(angles[0]) * cos(angles[2]) + cos(angles[0]) * sin(angles[1]) * sin(angles[2]);
    rotMat[6] = sin(angles[1]);
    rotMat[7] = -cos(angles[1]) * sin(angles[0]);
    rotMat[8] = cos(angles[0]) * cos(angles[1]);
} 

/**
 * Multiplies two matrices with arbitrary dimensions
 * Matrix A has dimensions m x p
 * Matrix B has dimensions q x n
 * p == q is required
 * @param matResult The resulting matrix after multiplication
 */
void matrixMultip(double* matA, double* matB, double* matResult, int m, int p, int q, int n) {
    
    if (p == q) {
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                matResult[n * i + j] = 0;
			    for (int k = 0; k < p; k++)
				    matResult[n * i + j] = matResult[n * i + j] + matA[p * i + k] * matB[n * k + j];
            }
        }
    } else {
        Serial.println("Matrix dimensions do not match!");
    }
}