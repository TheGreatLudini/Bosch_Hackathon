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

//LEDStuff
const uint8_t LED_PIN(11);
const uint8_t LED_COUNT(5);
enum Leds : uint8_t {
    LedCenter,
    LedRight,
    LedUp,
    LedLeft,
    LedDown
};

// Motorcontrol
const uint8_t MOTOR_SPEED_PIN = 6;
const float ANGLE_DISPLACEMENT(8.0); // offset of the drilling axis to the buttom plate

const double MOTOR_ON_THESHOLD(0.3);
const double CENTER_LED_ON_THESHOLD(0.15);

// IMU Calibration in degree
// TODO to be determined
imu::Vector<3> xGlobal(1.0, 0.0, 0.0);
imu::Vector<3> yGlobal(0.0, 1.0, 0.0);
imu::Vector<3> zGlobal(0.0, 0.0, 1.0);
// imu::Vector<3> xGlobal(-0.05023, 0.01451, -0.99862);
// imu::Vector<3> yGlobal(0.01260, 0.00079, 0.01389);
// imu::Vector<3> zGlobal(0.99861, -0.01188, -0.05040);

// Current measurement
#define CURRENT_SENSE_PIN A6
#define VOLT_FOR_PIN A2
#define VOLT_BACK_PIN A7
const double GAIN(20.0);
const double SENSE_RESISTANCE(0.002);
const double CURRENT_FACTOR = REF_VOLTAGE / (1023 * GAIN * SENSE_RESISTANCE);
const double VOLTAGE_FACTOR = REF_VOLTAGE * 5.7 / 1023; // 47k und 10k Wiederstäde im Spannungsteiler

const uint16_t FILTERLENGTH(50);
const uint16_t VOLT_FILTERLENGTH(30);
const double VOLT_TRIGGER_TH(3.0);
const uint32_t TRIGGER_DEBOUNCE(20);

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
// BLE Battery Service

// BLEService angleService("aa461740-dc53-4624-97bd-0fee7b1212bb");
// ;

// // BLE Battery Level Characteristic
// //BLEUnsignedIntCharacteristic SetAngleChar("951a4e8a-16a8-46a7-8962-0d5dc72881b5", BLERead | BLEWrite);
// BLEIntCharacteristic SetAngleCharLR("951a4e8a-16a8-46a7-8962-0d5dc72881b5", BLERead | BLEWrite);
// BLEIntCharacteristic SetAngleCharUD("05d47d9d-6295-491d-81ac-375395100e1e", BLERead | BLEWrite);
// BLEFloatCharacteristic SendScalarLR("de58c0ab-e1ee-4e87-a578-b40af4f5822b", BLERead | BLEWrite | BLENotify);
// BLEFloatCharacteristic SendScalarUD("a71f3fc6-f97c-4659-9ca2-76a67dede2e3", BLERead | BLEWrite | BLENotify);
// BLEBoolCharacteristic CalibrateChar("5b880d68-b5b9-420c-a528-d21beb197155", BLERead | BLENotify);

// void ConnectHandler(BLEDevice central) {
//   // central connected event handler
//   Serial.print("Connected event, central: ");
//   Serial.println(central.address());
//   BLE.advertise();
// }

// void DisconnectHandler(BLEDevice central) {
//   // central disconnected event handler
//   Serial.print("Disconnected event, central: ");
//   Serial.println(central.address());
//   BLE.advertise();
// }
#endif