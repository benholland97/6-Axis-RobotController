#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <vector>
#include <list>
#include "Config.h"

using namespace std;

struct JointAngles {
    JointAngles(float* _a){
        for(int i=0; i<NO_ACTUATORS; ++i) {
            angles.push_back(_a[i]);
        }
    }
    JointAngles(vector<float> _a):angles(_a){};

    JointAngles(const JointAngles& _ja):angles(_ja.angles){};

    float operator[](int idx) {
        return angles[idx];
    }

    float getAngleDeg(int idx) {
        return (angles[idx] * 180) / M_PI;
    }

    vector<float> angles;

    friend ostream& operator<<(ostream& os, const JointAngles& p);



};

class ServoController {
public:

    ServoController();

    ServoController(int x);

    void addTargetPosition(float* _a);
    void addTargetPosition(JointAngles _ja);

    int getNumMoves(){
        return targetAngles.size();
    }

    int isTargetEmpty() {
        return targetAngles.empty();
    }

    bool setAngles(JointAngles ja);

    bool setAngle(double angle, int servo);

    int getNumServos();

    bool mechLimitsCheck(float* angles);

    bool moveOne();

    // double mapRXToPulse(int i,double rx);



private:
    void init(int x);
    bool boundsCheck(double angle, int servo);

    list<JointAngles> targetAngles;
    Adafruit_PWMServoDriver pwm;
    vector<pair<float,float>> servoLimits;
    vector<pair<float,float>> angleLimits;

    // int pulse_limits_min[NO_ACTUATORS] = {SERVO0MIN,SERVO0MIN,SERVO0MIN,SERVO0MIN,SERVO0MIN,SERVO0MIN};
    // int pulse_limits_max[NO_ACTUATORS] = {SERVO0MAX,SERVO0MAX,SERVO0MAX,SERVO0MAX,SERVO0MAX,SERVO0MAX};
    // int angle_limits_min[NO_ACTUATORS] = {-MG996R_MAX_ANGLE_2,-MG996R_MAX_ANGLE_2,-MG996R_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2};    
    // int angle_limits_max[NO_ACTUATORS] = {MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2,MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2};

    int numServos = 0;

};

#endif