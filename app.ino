#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "config.h"
    


Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event; 
float angle[] = {0, 0, 0};
    
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
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setAngle, RISING);
    Serial.println("Setup done");
}
    
void loop(void) 
{   
    bno.getEvent(&event);
    float xError = event.orientation.x - angle[0];
    if (abs(xError) > 270) {
        xError > 0 ? xError -= 360 : xError += 360;
    } else if (abs(xError) > 90) {
        xError > 0 ? xError -= 180 : xError += 180;
    }
    
    float yError = event.orientation.y - angle[1];
    delay(100);
    digitalWrite(LED_BUILTIN, abs(xError) < ANGLE_DISPLACEMENT && abs(yError) < ANGLE_DISPLACEMENT);
    double angles[3] = {event.orientation.x, event.orientation.y, event.orientation.z};
    double cartesian[3];
    double vecToRotate[3] = {1, 0, 0};
    getCartesian(angles, vecToRotate, cartesian);
    Serial.print("X: ");
    Serial.print(cartesian[0]);
    Serial.print("\t Y: ");
    Serial.print(cartesian[1]);
    Serial.print("\t Z: ");
    Serial.println(cartesian[2]);
}

void setAngle()
{
    Serial.println("Interrupt");
    sensors_event_t event2; 
    bno.getEvent(&event2);
    /*
    angle[0] = event.orientation.x;
    // angle[1] = event.orientation.y + 90.0;
    angle[1] = event.orientation.y;
    angle[2] = event.orientation.z;
    Serial.println("Angle was saved");
    Serial.print(angle[0]);
    Serial.print(angle[1]);
    Serial.print(angle[2]);*/
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