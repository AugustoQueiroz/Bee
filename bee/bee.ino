// #include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

static const int RXPin = 4, TXPin = 3;
SoftwareSerial softSerial(RXPin, TXPin);

void setup() {
    // put your setup code here, to run once:
    Wire.begin(); // Necessary for the RTC
    Serial.begin(38400); // Serial for the OBD reader
    // softSerial.begin(GPSBaud); // Serial for the GPS reader
}

void loop() {
    // put your main code here, to run repeatedly:
    OBDManager();
}

/*
 *  Misc. Functions
 */
void clearSerial(void) {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

/*
 *  OBD Functions
 */

typedef int (*parseFunc)(char*);

int allowedPIDs[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

parseFunc parseFuncs[] = { 
    NULL, 
    NULL, 
    NULL, 
    NULL, 
    parsePercentage, 
    parseTemperature, 
    parsePercentage, 
    parsePressure765, 
    parsePressure255, 
    parseRPM, 
    parseSpeed, 
    NULL, 
    parseTemperature, 
    parseAirFlowRate, 
    parsePercentage,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	parseSeconds,
 	NULL,
 	parseDistance,
 	NULL,
 	NULL,
 	NULL,
 	parsePercentage,
 	NULL,
 	parsePercentage,
 	parsePercentage,
 	parseCount,
 	parseDistance,
 	parsePressure255,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	parsePercentage,
 	parseTemperature,
 	parsePercentage,
 	parseMinutes,
 	NULL,
 	NULL,
 	NULL,
 	parsePercentage,
 	NULL,
 	NULL,
 	NULL,
 	NULL,
 	parsePercentage,
 	parsePercentage,
 	parseTemperature,
 	NULL
};

void getAllowedPIDs() {
    char* response;

    Serial.println("0100");
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[0] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[1] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("0120");
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[2] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[3] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("0140");
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[4] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[5] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("0160");
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[6] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[7] = ((int) response[2] << 8) | (int) response[3];
    
    Serial.println("0180");
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[8] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[9] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("01A0");
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[10] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[11] = ((int) response[2] << 8) | (int) response[3];
}

void OBDManager() {
    static char iPID = 0;

    if (allowedPIDs[iPID/16] & (1 << (16 - iPID%16))) {
        String PID = String(String("01") + String((iPID < 16) ? "0" : "") + String(iPID, HEX));
        
        int result = obdRead(PID);
        // Do stuff with the result
    }

    iPID++;
}

int obdRead(String PID) {
    char iPID = parsePID(PID); // Use the PID to know what function to use for parsing
    char* response;
    Serial.println(PID);

    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);

    if (iPID < 92 && parseFuncs[iPID] != NULL) {
        return parseFuncs[iPID](response);
    } else {
        return 0;
    }
}

char* getResponse(void) {
    char response[20];
    char inChar = 0;
    char index = 0;

    // Keep reading chars from OBD until receives a carriage return (\r)
    while (inChar != '\r') {
        if (Serial.available() > 0) { // There is something to be read on the Serial port
            if ((Serial.peek() == 0x3E) || (Serial.peek() == '\0') || (Serial.peek() == '\r')) {
                inChar = Serial.read();
                response[index] = '\0';
                index = 0;
            } else {
                inChar = Serial.read();
                response[index++] = inChar;
            }
        }
    }

    // Clean the Serial bugger if there is anything left
    clearSerial();

    // Parse the received string and return it
    return parseHex(response);
}

/*
 *  Parse Formulas from
 *  https://en.m.wikipedia.org/wiki/OBD-II_PIDs
 */

int parseTemperature(char* response) {
    char A = response[0];
    
    return A - 40;
}

int parsePercentage(char* response) {
    char A = response[0];

    return 100.0/255.0 * A;
}

int parsePressure255(char* response) {
    char A = response[0];

    return A;
}

int parsePressure765(char* response) {
    char A = response[0];

    return 3*A;
}

int parseRPM(char* response) {
    char A = response[0],
         B = response[1];

    return (256*A + B)/4;
}

int parseSpeed(char* response) {
    char A = response[0];

    return A;
}

int parseAirFlowRate(char* response) {
    char A = response[0],
         B = response[1];

    return (256*A + B)/100;
}

int parseSeconds(char* response) {
    char A = response[0],
         B = response[1];

    return 256*A + B;
}

int parseDistance(char* response) {
    char A = response[0],
         B = response[1];

    return 256*A + B;
}

int parseMinutes(char* response) {
    char A = response[0],
         B = response[1];

    return 256*A + B;
}

int parseCount(char* response) {
  char A = response[0];

  return A;
}

char* parseHex(char* data) {
    char tmp[3] = {' ', ' ', '\0'};
    static char response[4];
    int i, j;
    
    for (i = 0, j = 6; i < 4; i++, j += 3) {
        tmp[0] = data[j];
        tmp[1] = data[j+1];

        response[i] = strtol(tmp, NULL, 16);
    }

    return response;
}

char parsePID(String PID) {
    char tmp[3] = {PID[2], PID[3], '\0'};
    
    return strtol(tmp, NULL, 16);
}
