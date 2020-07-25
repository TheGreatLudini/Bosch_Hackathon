#ifndef CONFIG_H
#define CONFIG_H

#define CURRENT_SENSE_PIN A1

const uint8_t INTERRUPT_PIN = 2;
const uint32_t debounce(20); // 20 ms debounce time to prevent flickerining

const uint8_t LED_UP = 10;
const uint8_t LED_RIGHT = 9;
const uint8_t LED_DOWN = 6;
const uint8_t LED_LEFT = 5;

// Motorcontrol
const uint8_t MOTOR_SPEED_PIN = 3;

const float ANGLE_DISPLACEMENT(8.0); // offset of the drilling axis to the buttom plate

#endif