#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MatrixMath.h>

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
    
    bno.setExtCrystalUse(true);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setAngle, FALLING);
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
    Serial.print("/t Y: ");
    Serial.print(cartesian[1]);
    Serial.print("/t Z: ");
    Serial.println(cartesian[2]);
}

void setAngle()
{
    bno.getEvent(&event);

    angle[0] = event.orientation.x;
    angle[1] = event.orientation.y + 90.0;
    angle[2] = event.orientation.z;
    Serial.println("Angle was saved");
    Serial.print(angle[0]);
    Serial.print(angle[1]);
    Serial.print(angle[2]);
}

void getCartesian(double* angles, double* vecToRotate, double* cartesian) {
    uint8_t N = 3;
    mtx_type rotMat[N][N]; 
    getRot(angles, (mtx_type*) rotMat);
    Matrix.Multiply((mtx_type*) (mtx_type*)rotMat, angles, N, N, N, (mtx_type*)cartesian);
}

/**
 * conversion of euler values to cartesian with XYZ cnvention
 */
void getRot(double* angles, mtx_type* rotMat) {
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