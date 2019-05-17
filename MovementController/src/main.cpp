#include <Arduino.h> 
#include "Config.h"
#include "ServoController.h"
#include "SerialRx.h"

using namespace std;

void setup() {
    Serial.begin(9600);
    Serial.println("Initialising Robot Arm");
    ServoController sc = ServoController();
    SerialRX sRX = SerialRX(Serial);

    //main loop
    while(1) {
        float buffer[NO_ACTUATORS];
        if(sRX.recv(buffer)) {
            sc.addTargetPosition(buffer);
            // cout<<"<1>"<<"\n";
            // if(sc.isTargetEmpty()) cout<<"<1>\n";

        }
        if(!sc.isTargetEmpty()) {
            if(sc.moveOne()) {
                // cout<<"setting target angles"<<endl;
                cout<<"<1>\n";
            } else {
                cout<<"<0>";
            }
        }
    }
}

void loop() {

}


