#include "SerialRx.h"

// SerialRX::SerialRX(Stream& _s) {
//     newData = false;
//     stream(_s);
// }

bool SerialRX::recv2(float* a) {
    String input;
    //If any input is detected in arduino
    if(stream.available() > 0){
        //read the whole string until '\n' delimiter is read
        input = stream.readStringUntil('\n');
        stream.println("Received");
        stream.println(input);

        return true;
    }
    return false;
}



bool SerialRX::recv(float *a) {
    recvWithStartEndMarkers();
    if (newData == true) {
        // Serial.print("Arduino side: Raw data received:\n");
        // for(int i=0; i<NUM_CHARS; ++i) {
        //     Serial.print(receivedChars[i]);
        // }
        // // Serial.print(tempChars);
        // Serial.print("\n");
        strcpy(tempChars, receivedChars);
        // for(int i=0; i<NUM_CHARS; ++i) {
        //     stream.println(tempChars[i]);
        // }
        parseData(a);
        // showParsedData(a);
        newData = false;
        // for(int i=0; i<size; ++i) {
        //     a[i] = a[i];
        // }

        return true;
    }
    return false;
}

void SerialRX::flushBuffer() {
    while (stream.available() >0 ) {
        char a = stream.read();
        stream.println(a);
    }
}

void SerialRX::recvWithStartEndMarkers() {
    static bool recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    // memset(receivedChars, 0, sizeof(receivedChars));
    // memset(tempChars, 0, sizeof(tempChars));


    while (stream.available() > 0 && newData == false) {
        rc = stream.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                // Serial.print(rc);
                ndx++;
                if (ndx >= NUM_CHARS) {
                    ndx = NUM_CHARS - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                // Serial.print("\nArduino side: Raw data received ended - \n");
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            // Serial.print("Arduino side: Raw data received starting - \n");
            recvInProgress = true;
        }
    }
}

void SerialRX::parseData(float* a) {
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
    a[0] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[1] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[2] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[3] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[4] = atof(strtokIndx);     
    strtokIndx = strtok(NULL, ",");
    a[5] = atof(strtokIndx);    
    strtokIndx = strtok(NULL, ",");
    a[6] = atof(strtokIndx);     
}

void SerialRX::showParsedData(float* a) {
    for(int i=0; i<NO_ACTUATORS; ++i) {
        stream.print(a[i]);
        stream.print("\t");
    }
    stream.print("\n");
}