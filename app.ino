#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 

#include "config.h"
//#include "Motor.h"

// #define DEBUG_LOCAL_VECTOR
// #define DEBUG_MISSALIGN
// #define DEBUG_MOTOR
#define DEBUG_TOTAL_ERROR
#define DEBUG_ERROR_DIR
//#define DEBUG_MISSALIGN


Adafruit_BNO055 bno = Adafruit_BNO055(55);

imu::Vector<3> wallNormal(0.0, 0.0, 0.0);
imu::Vector<3> wall_X(0.0, 0.0, 0.0);
imu::Vector<3> wall_Y(0.0, 0.0, 0.0);
double localLeftRightError = 0.0;
double localUpDownError = 0.0;

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

bool initialize = false;
bool initialize = true;
bool preciceInitialize = false;
bool initGuard = false;
uint32_t interruptTime(0);
uint8_t interruptCounter(0);

//Motor myMotor = new Motor(MOTOR_FOR_DIR_PIN, MOTOR_BACK_DIR_PIN, MOTOR_SPEED_PIN);

void setup(void) 
{
    Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }

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


    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_UP, OUTPUT);
    pinMode(LED_RIGHT, OUTPUT);
    pinMode(LED_DOWN, OUTPUT);
    pinMode(LED_LEFT, OUTPUT);

    pinMode(MOTOR_SPEED_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setAngle, FALLING);
    Serial.println("Setup done");

    // HTTP server setup:
    serverSetup();
}

void loop(void) 
{   
    serverLoop();
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
    localLeftRightError = wallNormal.dot(yLocal);
    localUpDownError = wallNormal.dot(zLocal);

    // LED on if the drilling angle is correct
    digitalWrite(LED_BUILTIN, abs(localLeftRightError) < ANGLE_DISPLACEMENT && abs(localUpDownError) < ANGLE_DISPLACEMENT);

    double localErrorTotal = sqrt(pow(localUpDownError, 2) + pow(localLeftRightError, 2));
    uint8_t motorSpeed(0);
    if (localErrorTotal < MOTOR_ON_THESHOLD) {
        localErrorTotal = localErrorTotal * 100 / 30 * 255;
        motorSpeed = 255 - localErrorTotal;
    }
    analogWrite(MOTOR_SPEED_PIN, motorSpeed);
    

    #ifdef DEBUG_MOTOR
    Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);
    #endif

    
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
    //delay(100);


}

void setAngle()
{
    if (abs(millis() - interruptTime) > 500) {
        interruptCounter = 0;
    }
    if (interruptCounter == 0) {
        interruptCounter++;
        preciceInitialize = true;
        initGuard = true;
    } else if (abs(millis() - interruptTime) > debounce) {
        interruptCounter = 0;
        preciceInitialize = false;
        initialize = true;
    }

    interruptTime = millis();
    Serial.println("Initializing");
}

/**
 * initializes the Wall coordinates assuming the drill
 * is hold against a wall
 */
void preciceInit() {
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

void serverSetup() {
    Serial.println("Access Point Web Server");
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true);
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Please upgrade the firmware");
    }

    // by default the local IP address of will be 192.168.4.1
    // you can override it with the following:
    // WiFi.config(IPAddress(10, 0, 0, 1));

    // print the network name (SSID);
    Serial.print("Creating access point named: ");
    Serial.println(ssid);

    // Create open network. Change this line if you want to create an WEP network:
    status = WiFi.beginAP(ssid);
    if (status != WL_AP_LISTENING) {
        Serial.println("Creating access point failed");
        // don't continue
        while (true);
    }

    // wait 10 seconds for connection:
    delay(10000);

    // start the web server on port 80
    server.begin();

    // you're connected now, so print out the status
    printWiFiStatus();
}

void serverLoop() {
    // compare the previous status to the current status
    if (status != WiFi.status()) {
        // it has changed update the variable
        status = WiFi.status();

        if (status == WL_AP_CONNECTED) {
        // a device has connected to the AP
            Serial.println("Device connected to AP");
        } else {
        // a device has disconnected from the AP, and we are back in listening mode
            Serial.println("Device disconnected from AP");
        }
    }
    
    WiFiClient client = server.available();   // listen for incoming clients

    if (client) {                             // if you get a client,
        Serial.println("new client");           // print a message out the serial port
        String currentLine = "";                // make a String to hold incoming data from the client
        while (client.connected()) {            // loop while the client's connected
            if (client.available()) {             // if there's bytes to read from the client,
                char c = client.read();             // read a byte, then
                Serial.write(c);                    // print it out the serial monitor
                if (c == '\n') {                    // if the byte is a newline character

                // if the current line is blank, you got two newline characters in a row.
                // that's the end of the client HTTP request, so send a response:
                if (currentLine.length() == 0) {/*
                    // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                    // and a content-type so the client knows what's coming, then a blank line:
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/html");
                    client.println();

                    // the content of the HTTP response follows the header:
                    client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
                    client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");

                    // The HTTP response ends with another blank line:
                    client.println();
                    // break out of the while loop:
                    */
                    break;
                } else {      // if you got a newline, then clear currentLine:
                    currentLine = "";
                }
                } else if (c != '\r') {    // if you got anything else but a carriage return character,
                    currentLine += c;      // add it to the end of the currentLine
                }

                // Check to see if the client request was "GET /H" or "GET /L":
                if (currentLine.endsWith("GET /H")) {
                    digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
                }
                if (currentLine.endsWith("GET /L")) {
                    digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
                }
                if (currentLine.endsWith("GET /testValue")) {
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/plain");
                    client.println();    

                    client.print("geht es?????");
                    client.println();
                }
                if (currentLine.endsWith("GET /getLocalTilt")) {
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/plain");
                    client.println();    

                    client.print(to_string(localLeftRightError) + "_" + to_string(localUpDownError));
                    client.println();
                }
            }
        }
        // close the connection:
        client.stop();
        Serial.println("client disconnected");
    }
}

void printWiFiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print where to go in a browser:
    Serial.print("To see this page in action, open a browser to http://");
    Serial.println(ip);

}