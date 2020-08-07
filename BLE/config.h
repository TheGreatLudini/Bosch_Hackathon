#ifndef CONFIG_H
#define CONFIG_H
#include <Adafruit_NeoPixel.h>


#if defined(ARDUINO_AVR_NANO)       
    #define BOARD_NANO
    const float REF_VOLTAGE(5.0);
#else
    const float REF_VOLTAGE(3.3);
#endif
const uint8_t INTERRUPT_PIN = 2;
const uint32_t LONG_PRESS_TIME(400);


//LEDStuff
const uint8_t LED_PIN(11);
const uint8_t LED_COUNT(6);
const float CENTER_LED_DECREASE(4.5);
const float LED_INCREASE(5.0);
enum Leds : uint8_t {
    LedTop,
    LedCenter,
    LedRight,
    LedUp,
    LedLeft,
    LedDown  
};

// Motorcontrol
const uint8_t MOTOR_SPEED_PIN = 6;
const float ANGLE_DISPLACEMENT(8.0); // offset of the drilling axis to the buttom plate

const double MOTOR_ON_THESHOLD(0.14);
const int MOTOR_SPEED_SENSITIVITY(1200); // sensitivity of motor to angle displacement, the higher the more sensitive
const double CENTER_LED_ON_THESHOLD = MOTOR_ON_THESHOLD;

// IMU Calibration in degree
// TODO to be determined
imu::Vector<3> xGlobal(1.0, 0.0, 0.0);
imu::Vector<3> yGlobal(0.0, 1.0, 0.0);
imu::Vector<3> zGlobal(0.0, 0.0, 1.0);

// Current measurement
#define CURRENT_SENSE_PIN A6
#define VOLT_FOR_PIN A2
#define VOLT_BACK_PIN A7
const double GAIN(20.0);
const double SENSE_RESISTANCE(0.002);
const double CURRENT_FACTOR = REF_VOLTAGE / (1023 * GAIN * SENSE_RESISTANCE);
const double VOLTAGE_FACTOR = REF_VOLTAGE * 5.7 / 1023; // 47k und 10k Wiederstäde im Spannungsteiler

const uint16_t DERRIV_LENGTH(10);
const double CURRENT_TH(1.0);
const double DERR_CURRENT_TH(1.2);

#define TRIGGER_PIN A0
const uint16_t FILTERLENGTH(15);
const uint16_t DERR_FILTERLENGTH(10);
const uint16_t CURR_FILTERLENGTH(30);
const uint16_t VOLT_FILTERLENGTH(10);
const uint16_t VOLT_TRIGGER_TH(500); // analog read value (0-1023) for trigger threshold, the higher the less sensitive
const uint32_t TRIGGER_DEBOUNCE(20);
const uint32_t TRIGGER_MAX_DUR(200); // maximum time between trigger push and release
const uint32_t TRIGGER_MIN_DUR(50); // minimum time between trigger push and release

const uint32_t debounce(10); // 20 ms debounce time to prevent flickerining

//Functions
void setAngle();
double CurrentMeasurment();
void setLeds(double localLeftRightError, double localUpDownError, double localErrorTotal);
void preciceInit();
void Init();
imu::Vector<3> RotDir(double angle, imu::Vector<3> rotAxis, imu::Vector<3> VecToRotate);
void matrixMultip(double* matA, double* matB, double* matResult, int m, int p, int q, int n);
unsigned int encodeValue(double leftRight, double upDown);

#endif