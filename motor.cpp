#include "motor.h"

Motor::Motor(uint8_t forDirPin, uint8_t backDirPin, uint8_t speedPin) {
    MOTOR_SPEED_PIN = speedPin;
    MOTOR_BACK_DIR_PIN = backDirPin;
    MOTOR_FOR_DIR_PIN = forDirPin;

    pinMode(MOTOR_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_BACK_DIR_PIN, OUTPUT);
    pinMode(MOTOR_FOR_DIR_PIN, OUTPUT);
}

void Motor::setDir(bool forDforwardir){
    this->forward = forward;
    digitalWrite(MOTOR_FOR_DIR_PIN, forward);
    digitalWrite(MOTOR_BACK_DIR_PIN, !forward);
}

void Motor::setSpeed(uint8_t speed) {
    this->speed = speed;
    analogWrite(MOTOR_SPEED_PIN, speed);
}

uint8_t Motor::getSpeed() {
    return speed;
}

bool Motor::getDir(){
    return forward;
}