#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    public: 
        Motor(uint8_t forDirPin, uint8_t backDirPin, uint8_t speedPin);
        void setDir(bool forward);
        void setSpeed(uint8_t speed);
        uint8_t getSpeed();
        bool getDir();
    private:
        uint8_t MOTOR_SPEED_PIN;
        uint8_t MOTOR_BACK_DIR_PIN;
        uint8_t MOTOR_FOR_DIR_PIN;

        bool forward;
        uint8_t speed;

}
#endif