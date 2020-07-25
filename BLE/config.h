#ifndef CONFIG_H
#define CONFIG_H

const uint8_t INTERRUPT_PIN = 2;
const uint8_t LED_UP = 10;
const uint8_t LED_RIGHT = 9;
const uint8_t LED_DOWN = 6;
const uint8_t LED_LEFT = 5;


const uint8_t MOTOR_SPEED_PIN = 3;
const uint8_t MOTOR_BACK_DIR_PIN = 4;
const uint8_t MOTOR_FOR_DIR_PIN = 5;
const uint8_t DRILL_ANGLE_OFFSET = 8; // 8 deg
const double MOTOR_ON_THESHOLD(0.3);

const float ANGLE_DISPLACEMENT = 10.0;

const uint32_t debounce(10); // 20 ms debounce time to prevent flickerining

// BLE Battery Service
BLEService angleService("Angle_service");

// BLE Battery Level Characteristic
BLEUnsignedIntCharacteristic SetAngleChar("SetAngleChar", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic GetAngleChar("GetAngleChar", BLERead);
BLEUnsignedIntCharacteristic CalibrateChar("CalibrateChar", BLERead | BLENotify);




#endif