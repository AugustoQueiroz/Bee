#include "stm32f103c8t6.h"
#include "mbed.h"
#include "TinyGPS.h"
#include <string>
#include "main.h"
#include "pins.h"
#include "config.h"


/*
 *  Serial Init
 */
// TX, RX, BAUD
RawSerial telitSerial(TELIT_TX, TELIT_RX, TELIT_BAUD);
RawSerial obdSerial(OBD_RX, OBD_TX, OBD_BAUD);
RawSerial gpsSerial(GPS_RX, GPS_TX, GPS_BAUD);

/*
 *  GPS Init
 */
TinyGPS gps;
float distanceSinceLastMessage = 0;
float latitude = TinyGPS::GPS_INVALID_F_ANGLE;
float longitude = TinyGPS::GPS_INVALID_F_ANGLE;


DigitalOut statusLed(LED1);

/*
 *  OBD Init
 */
uint16_t allowedPIDs[12] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

typedef int16_t (*parseFunc)(char*);

uint8_t packageBuilderState = 0;

const parseFunc parseFuncs[] = { 
    NULL, 
    parseMIL, 
    NULL, 
    NULL, 
    parsePercentage, 
    parseTemperature, 
    parsePercentage,
    parsePercentage,
    parsePercentage,
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

// Setup SD


/*void onReceive(){
    //pc.printf("datain\n");
    pc.putc(telitSerial.getc());
}*/


int main(){

    getAllowedPIDs();


    //init SD

    GPSManager();
    OBDManager();
    PackageManager();


    //pc.printf("Hello World !\n");
    /*char at[3] = {'A','T','\0'};
    char command[5] = {'+','G','M','I','\0'};
    char terminator[2] = {0xD, '\0'};
    char message[10];*/
    //char buffer[100];
    //telitSerial.attach(&onReceive);
    /*std::string at = "AT+";
    std::string at1 = "GMI\r";
    for (int i = 0; at[i]!='\0'; ++i){
        //pc.putc(at[i]);
        telitSerial.putc(at[i]);
    }
    for (int i = 0; at1[i]!='\0'; ++i){
        //pc.putc(at[i]);
        telitSerial.putc(at1[i]);
    }*/
    /*strcat(at,message);
    strcat(command,message);
    strcat(terminator,message);*/
    /*i=0;
    while (message[i]!='\0'){
        pc.printf("%s\n", message[i]);
        telitSerial.printf("%s", message[i]);
        i++;
    }*/
    //telitSerial.printf("%s", message);
    /*while(telitSerial.readable()){
        pc.putc(telitSerial.getc());
    }*/
}

void PackageManager() {
    static unsigned long lastReset = millis();

    ///Serial.println(F("doing SD stuff"));

    if (millis() - lastReset > SEND_INTERVAL * 1000) {
        switch (packageBuilderState) {
        case 0:
            startPackage(); // Mem - 3%
            saveTimestamp(); // Mem - 6%
            savePosition(); // Mem - 0%
            saveDistanceTravelled(); // Mem - 2%
            startPIDs(); // Mem - 3%
            packageBuilderState++;
            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            endPIDs(); // Mem - 2%
            endPackage(); // Mem - 3%
            //sendPackage(); // Mem - 1%
            lastReset = millis();
            distanceSinceLastMessage = 0;
            packageBuilderState = 0;
            break;
        }
    } else {
        //Serial.println(SEND_INTERVAL * 1000 - (millis() - lastReset));
    }
}

void startPackage() {
    package = SD.open(F("package"), FILE_WRITE);

    Serial.println(F("starting package"));
    if (package) {
        //package.write('h');
        //package.println("\"msg\" : {");
    } else {
        //Serial.println(F("error starting package"));
    }
}

void GPSManager() {
    static float lastLatitude = 0;
    static float lastLongitude = 0;

    //Serial.println(F("doing GPS stuff"));

    while (obdSerial.readable() > 0) {
        if (gps.encode(obdSerial.getc())) {            
            gps.f_get_position(&latitude, &longitude);
            
            //Serial.print("lat: "); Serial.print(latitude, 6); Serial.print(" lng: "); Serial.println(longitude, 6);

            if (lastLatitude != 0) {
                distanceSinceLastMessage += distanceBetweenCoordinates(latitude, longitude, lastLatitude, lastLongitude);
            }
            lastLatitude = latitude;
            lastLongitude = longitude;
        }
    }
}

float distanceBetweenCoordinates(float latitude1, float longitude1, float latitude2, float longitude2) {
    float R = 6371e3;
    float phi1 = toRadians(latitude1);
    float phi2 = toRadians(latitude2);
    float deltaPhi = toRadians(latitude2 - latitude1);
    float deltaLambda = toRadians(longitude2 - longitude1);

    float a = sin(deltaPhi/2) * sin(deltaPhi/2) + cos(phi1)*cos(phi2)*sin(deltaLambda/2)*sin(deltaLambda/2);
    float c = 2*atan2(sqrt(a), sqrt(1-a));

    return R*c;
}

void savePID(std::string PID, int16_t val) {
    //Serial.println("saving PID");
    
    /*if (package) {
        package.print(F("\t\t\"")); package.print(PID);package.print(F("\": "));
        package.print(val);
        package.println(F(","));
    }*/
}

char parsePID(std::string PID) {
    char tmp[3] = {PID[2], PID[3], '\0'};
    
    return strtol(tmp, NULL, 16);
}

int16_t obdRead(std::string PID) {
    int16_t iPID = parsePID(PID); // Use the PID to know what function to use for parsing
    char* response;
    obdSerial.printf("%s", PID.c_str());
    //Serial.print(F("Reading ")); Serial.println((int)iPID);

    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);

    if (parseFuncs[iPID] != NULL) {
        parseFunc pF = parseFuncs[iPID];
        return pF(response);
    } else {
        return 0;
    }
}

void OBDManager() {
    static uint8_t iPID = 0;

    //Serial.println(F("doing OBD stuff"));

    if (allowedPIDs[iPID/16] & (1 << (16 - iPID%16))) {
        char temp[3];
        sprintf(temp, "%X", iPID);
    
        std::string temp2 = ((iPID < 16) ? ("0") : (""));
        std::string PID = "01" + temp2 + temp;
        int16_t result = obdRead(PID);

        // Do stuff with the result
        if (packageBuilderState == 2)
            savePID(PID, result);
    }

    iPID++;

    if (iPID > 0xC4) {
        iPID = 0;

        if (packageBuilderState == 1 || packageBuilderState == 2) {
            packageBuilderState++;
            //Serial.println("Starting to save PIDs/PIDs Saved");
        }
    }
}


char* parseHex(char* data) {
    char tmp[3] = {' ', ' ', '\0'};
    static char response[4];
    uint8_t i, j;
    
    for (i = 0, j = 6; i < 4; i++, j += 3) {
        tmp[0] = data[j];
        tmp[1] = data[j+1];

        response[i] = strtol(tmp, NULL, 16);
    }

    return response;
}

void clearOBDSerial(void) {
    while (obdSerial.readable() > 0) {
        obdSerial.getc();
    }
}

char* getResponse(void) {
    static char response[20];
    char inChar = 0;
    uint8_t index = 0;

    // Keep reading chars from OBD until receives a carriage return (\r)
    while (inChar != '\r') {
        if (obdSerial.readable() > 0) { // There is something to be read on the Serial port
            inChar = obdSerial.getc();
            if ((inChar == 0x3E) || (inChar == '\0') || (inChar == '\r')) {
                response[index] = '\0';
                index = 0;
            } else {
                response[index++] = inChar;
            }
        }
    }

    // Clean the Serial bugger if there is anything left
    clearOBDSerial();

    // Parse the received string and return it
    return parseHex(response);
}

void getAllowedPIDs() {
    char* response;

    obdSerial.printf("0100\n");
    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);
    allowedPIDs[0] = ((uint16_t) response[0] << 8) | (uint16_t) response[1];
    allowedPIDs[1] = ((uint16_t) response[2] << 8) | (uint16_t) response[3];

    obdSerial.printf("0120\n");
    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);
    allowedPIDs[2] = ((uint16_t) response[0] << 8) | (uint16_t) response[1];
    allowedPIDs[3] = ((uint16_t) response[2] << 8) | (uint16_t) response[3];

    obdSerial.printf("0140\n");
    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);
    allowedPIDs[4] = ((uint16_t) response[0] << 8) | (uint16_t) response[1];
    allowedPIDs[5] = ((uint16_t) response[2] << 8) | (uint16_t) response[3];

    obdSerial.printf("0160\n");
    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);
    allowedPIDs[6] = ((uint16_t) response[0] << 8) | (uint16_t) response[1];
    allowedPIDs[7] = ((uint16_t) response[2] << 8) | (uint16_t) response[3];
    
    obdSerial.printf("0180\n");
    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);
    allowedPIDs[8] = ((uint16_t) response[0] << 8) | (uint16_t) response[1];
    allowedPIDs[9] = ((uint16_t) response[2] << 8) | (uint16_t) response[3];

    obdSerial.printf("01A0\n");
    response = getResponse();
    wait(1);
    response = getResponse();
    wait(0.2);
    allowedPIDs[10] = ((uint16_t) response[0] << 8) | (uint16_t) response[1];
    allowedPIDs[11] = ((uint16_t) response[2] << 8) | (uint16_t) response[3];
}

int16_t parseMIL(char* response) {
    char A = response[0];

    return A & (1 << 7);
}

int16_t parseTemperature(char* response) {
    char A = response[0];
    
    return A - 40;
}

int16_t parsePercentage(char* response) {
    char A = response[0];

    return 100.0/255.0 * A;
}

int16_t parsePressure255(char* response) {
    char A = response[0];

    return A;
}

int16_t parsePressure765(char* response) {
    char A = response[0];

    return 3*A;
}

int16_t parseRPM(char* response) {
    return (256*response[0] + response[1])/4;
}

int16_t parseSpeed(char* response) {
    return response[0];
}

int16_t parseAirFlowRate(char* response) {
    return (256*response[0] + response[1])/100;
}

int16_t parseSeconds(char* response) {
    return 256*response[0] + response[1];
}

int16_t parseDistance(char* response) {
    return 256*response[0] + response[1];
}

int16_t parseMinutes(char* response) {
    return 256*response[0] + response[1];
}

int16_t parseCount(char* response) {
  return response[0];
}