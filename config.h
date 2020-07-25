#ifndef CONFIG_H
#define CONFIG_H

#if defined(ARDUINO_AVR_NANO)       
    #define BOARD_NANO
    const float REF_VOLTAGE(5.0);
#else
    const float REF_VOLTAGE(3.3);
#endif


const uint8_t INTERRUPT_PIN = 2;

const uint8_t LED_UP = 10;
const uint8_t LED_RIGHT = 9;
const uint8_t LED_DOWN = 6;
const uint8_t LED_LEFT = 5;

// Motorcontrol
const uint8_t MOTOR_SPEED_PIN = 3;
const uint8_t MOTOR_BACK_DIR_PIN = 4;
const uint8_t MOTOR_FOR_DIR_PIN = 5;
const uint8_t DRILL_ANGLE_OFFSET = 8; // 8 deg
const double MOTOR_ON_THESHOLD(0.3);

// Current measurement
#define CURRENT_SENSE_PIN A1
const double GAIN(20.0);
const double SENSE_RESISTANCE(0.002);
const double CURRENT_FACTOR = REF_VOLTAGE / (1023 * GAIN * SENSE_RESISTANCE);
const uint16_t FILTERLENGTH(50);

const uint32_t debounce(10); // 20 ms debounce time to prevent flickerining

const float ANGLE_DISPLACEMENT(8.0); // offset of the drilling axis to the buttom plate

#endif