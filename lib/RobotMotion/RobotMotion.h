#ifndef ROBOTMOTION
#define ROBOTMOTION

#include "Arduino.h"
#include "Wire.h"

class RobotMotion
{
    private:
        int ENA;
        int ENB;

        int IN1;
        int IN2;

        int IN3;
        int IN4;

        int speed;

    public:
        RobotMotion(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4);

        void setSpeed(int speed);
        
        void Stop();
        void Forward();
        void Backward();
        void Left();
        void Right();
};

#endif