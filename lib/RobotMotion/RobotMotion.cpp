#include "RobotMotion.h"

RobotMotion::RobotMotion(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4)
{
    this->ENA = ENA;
    this->ENB = ENB;

    this->IN1 = IN1;
    this->IN2 = IN2;

    this->IN3 = IN3;
    this->IN4 = IN4;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void RobotMotion::setSpeed(int speed)
{
    if(speed > 255)
        speed = 255;
    else if(speed < 0)
        speed = 0;
    
    this->speed = speed;
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void RobotMotion::Stop()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void RobotMotion::Forward()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void RobotMotion::Backward()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void RobotMotion::Left()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void RobotMotion::Right()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}