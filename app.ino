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
    
    bno.setExtCrystalUse(true);
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setAngle, FALLING);
}
    
void loop(void) 
{   
    bno.getEvent(&event);
    float xError = event.orientation.x - angle[0];
    float yError = event.orientation.y - angle[1];
    Serial.print("X: ");
    Serial.print(event.orientation.x - angle[0]);
    Serial.print("Y: ");
    Serial.print(event.orientation.y - angle[1]);
    digitalWrite(LED_BUILTIN, abs(xError) < ANGLE_DISPLACEMENT && abs(yError) < ANGLE_DISPLACEMENT);

    
}

void setAngle()
{
    bno.getEvent(&event);

    angle[0] = event.orientation.x;
    angle[1] = event.orientation.y + 90.0;
    angle[2] = event.orientation.z;
}