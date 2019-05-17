#include "ServoController.h"

ServoController::ServoController() {
    init(NO_ACTUATORS);
}

ServoController::ServoController(int x) {
    init(x);
}

void ServoController::init(int x) {
    Serial.println("Initialising servo controller");
    pwm = Adafruit_PWMServoDriver();
    numServos = x;
    // servoLimits.push_back(make_pair(SERVO0MIN,SERVO0MAX));
    // servoLimits.push_back(make_pair(SERVO1MIN,SERVO1MAX));
    // servoLimits.push_back(make_pair(SERVO2MIN,SERVO2MAX));
    // servoLimits.push_back(make_pair(SERVO3MIN,SERVO3MAX));
    // servoLimits.push_back(make_pair(SERVO4MIN,SERVO4MAX));
    // servoLimits.push_back(make_pair(SERVO5MIN,SERVO5MAX));
    // servoLimits.push_back(make_pair(SERVO6MIN,SERVO6MAX));

    angleLimits.push_back(make_pair(-MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2));
    angleLimits.push_back(make_pair(-MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2));
    angleLimits.push_back(make_pair(-MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2));
    angleLimits.push_back(make_pair(-MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2));
    angleLimits.push_back(make_pair(-MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2));
    angleLimits.push_back(make_pair(-MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2));
    angleLimits.push_back(make_pair(-MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2));

    pwm.begin();
    pwm.setPWMFreq(50);  // Digital servos run at ~50 Hz updates
}

bool ServoController::setAngles(JointAngles ja) {
    for(int i=0; i<(int)ja.angles.size(); ++i) {
        if(!setAngle(ja.angles[i],i)) {
            cout<<"Failure setting angle "<<ja.angles[i]<<" to servo "<<i;
            return false;
        };
    }
    return true;
}

bool ServoController::setAngle(double angle, int servo) {
    if(boundsCheck(angle,servo)) {
        double pulselength;
        double offset;
        switch(servo) {
            case 0:
                pulselength = map(angle+SERVO0_OFFSET,angleLimits[0].first,angleLimits[0].second,SERVO0MIN, SERVO0MAX);
                offset = pulselength - 289;
                break;
            case 1:
                pulselength = map(-angle+SERVO1_OFFSET,angleLimits[1].first,angleLimits[1].second,SERVO1MIN, SERVO1MAX);
                offset = pulselength - 237;
                break;
            case 2:
                pulselength = map(angle+SERVO2_OFFSET,angleLimits[2].first,angleLimits[2].second,SERVO2MIN, SERVO2MAX);
                offset = pulselength - 277;
                break;
            case 3: 
                pulselength = map(angle+SERVO3_OFFSET,angleLimits[3].first,angleLimits[3].second,SERVO3MIN, SERVO3MAX);
                offset = pulselength - 277;
                break;
            case 4:
                pulselength = map(angle+SERVO4_OFFSET,angleLimits[4].first,angleLimits[4].second,SERVO4MIN, SERVO4MAX);
                offset = pulselength - 267;                
                break;
            case 5:
                pulselength = map(angle+SERVO5_OFFSET,angleLimits[5].first,angleLimits[5].second,SERVO5MIN, SERVO5MAX);
                offset = pulselength - 289;                
                break;
            case 6:
                pulselength = angle == 1? GRIPPER_CLOSE:GRIPPER_OPEN;
                // pulselength = map(angle,angleLimits[6].first,angleLimits[6].second,SERVO6MIN, SERVO6MAX);
                // offset = pulselength - 289;                
                break;
            default:
                pulselength = -1;
                break;
        }
        if (pulselength<0) {
            return false;
        }
        // cout<<"Offset :"<<offset<<" Servo: "<<servo<<"\n";
        pwm.setPWM(servo,0,pulselength);
        return true;
    }
    return false;
}


int ServoController::getNumServos() {return numServos;}

bool ServoController::boundsCheck(double angle, int servo) {
    if(servo>numServos-1 || servo<0) {
        return false;
    }
    int iAngle = angle;
    if(iAngle > angleLimits[servo].second || iAngle < angleLimits[servo].first) {
        return false;
    }
    return true;
}


bool ServoController::mechLimitsCheck(float* pAngles) {
    for(int i=0; i<NO_ACTUATORS; ++i) {
        if(!boundsCheck(pAngles[i],i)) {
            return false;
        }
    }
    return true;
}

void ServoController::addTargetPosition(float* _a){
    targetAngles.push_back(JointAngles(_a));
}

void ServoController::addTargetPosition(JointAngles _ja){
    targetAngles.push_back(_ja);
}

bool ServoController::moveOne() {
    // cout<<"Target angles "<< targetAngles.front();
    bool ret = setAngles(targetAngles.front());
    // cout<<"\tResult :"<<ret<<"\n";
    targetAngles.pop_front();
    return ret;
}


ostream& operator<<(ostream& os, const JointAngles& _ja) {
    JointAngles ja(_ja);
    os <<"{ 0: " <<ja[0] << "\t 1: " << ja.getAngleDeg(1) << "\t 2: "<< ja.getAngleDeg(2) <<"\t 3: "
        <<ja.getAngleDeg(3)<<"\t 4: "<<ja.getAngleDeg(4)<<"\t 5: "<<ja.getAngleDeg(5)<<"\t 6: "<<ja.getAngleDeg(6)<<"} ";
	return os;
}


