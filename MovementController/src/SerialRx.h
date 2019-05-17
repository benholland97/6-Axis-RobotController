#ifndef SERIALRX_H
#define SERIALRX_H

#include "Config.h"
#include <Arduino.h>
#include <HardwareSerial.h> 



class SerialRX {
public:
    SerialRX(Stream& _s):stream(_s) {
        newData = false;
    }

    bool recv(float *a);

    bool recv2(float *a);

    void flushBuffer();



private:
    Stream &stream;
    float recBuffer[NO_ACTUATORS];

    void recvWithStartEndMarkers();
    void parseData(float* a);
    void showParsedData(float* a);

    // static const byte numChars = 32;
    char receivedChars[NUM_CHARS];
    char tempChars[NUM_CHARS];
    bool newData;

};


#endif