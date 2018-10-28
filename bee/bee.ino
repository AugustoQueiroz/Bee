#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

/*
 *  GPS Initialiations
 */

#define GPSBaud 9600

TinyGPS gps;
#define RXPin 4
#define TXPin 3
SoftwareSerial softSerial(RXPin, TXPin);

float distanceSinceLastMessage = 0;
float latitude = TinyGPS::GPS_INVALID_F_ANGLE;
float longitude = TinyGPS::GPS_INVALID_F_ANGLE;

/*
 *  OBD Initializations
 */

typedef int (*parseFunc)(char*);

unsigned int allowedPIDs[12] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

SoftwareSerial obdSerial(5, 6);

/*
 *  Package Initializations
 */

//File history;
File package;

byte packageBuilderState = 0;

#define SEND_INTERVAL 10 // Interval between messages in seconds

void setup() {
    // put your setup code here, to run once:
    softSerial.begin(GPSBaud); // Serial for the GPS reader
    obdSerial.begin(38400);

    Serial.begin(9600); // Hardware Serial

    getAllowedPIDs();

    if (!SD.begin(8)) {
        //Serial.println(F("SD initialization failed."));

        while (1);
    }
    Serial.println(F("SD successfully initiated."));

    SD.remove(F("package"));
    //SD.remove(F("history"));

    package = SD.open(F("package"), FILE_WRITE);
    package.close();

    //history = SD.open(F("history"), FILE_WRITE);
    //history.close();
}

void loop() {
    // put your main code here, to run repeatedly:
    //RTCManager();
    GPSManager();
    OBDManager();
    PackageManager();
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
 *  RTC Functions
 */

void RTCManager() {
    switch (0) {
    case 0:
        //displayTime();
        break;
    }
}

/*
 *  GPS Functions
 */

void GPSManager() {
    static float lastLatitude = 0;
    static float lastLongitude = 0;

    //Serial.println(F("doing GPS stuff"));

    while (softSerial.available() > 0) {
        if (gps.encode(softSerial.read())) {            
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

float toRadians(float angle) {
    return angle*PI/180;
}

/*
 *  OBD Functions
 */

const parseFunc parseFuncs[] PROGMEM = { 
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

void getAllowedPIDs() {
    char* response;

    obdSerial.println(F("0100"));
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[0] = ((unsigned int) response[0] << 8) | (unsigned int) response[1];
    allowedPIDs[1] = ((unsigned int) response[2] << 8) | (unsigned int) response[3];

    obdSerial.println(F("0120"));
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[2] = ((unsigned int) response[0] << 8) | (unsigned int) response[1];
    allowedPIDs[3] = ((unsigned int) response[2] << 8) | (unsigned int) response[3];

    obdSerial.println(F("0140"));
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[4] = ((unsigned int) response[0] << 8) | (unsigned int) response[1];
    allowedPIDs[5] = ((unsigned int) response[2] << 8) | (unsigned int) response[3];

    obdSerial.println(F("0160"));
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[6] = ((unsigned int) response[0] << 8) | (unsigned int) response[1];
    allowedPIDs[7] = ((unsigned int) response[2] << 8) | (unsigned int) response[3];
    
    obdSerial.println(F("0180"));
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[8] = ((unsigned int) response[0] << 8) | (unsigned int) response[1];
    allowedPIDs[9] = ((unsigned int) response[2] << 8) | (unsigned int) response[3];

    obdSerial.println(F("01A0"));
    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);
    allowedPIDs[10] = ((unsigned int) response[0] << 8) | (unsigned int) response[1];
    allowedPIDs[11] = ((unsigned int) response[2] << 8) | (unsigned int) response[3];
}

void OBDManager() {
    static unsigned char iPID = 0;

    //Serial.println(F("doing OBD stuff"));

    if (allowedPIDs[iPID/16] & (1 << (16 - iPID%16))) {
        String PID = String(String(F("01")) + String((iPID < 16) ? F("0") : F("")) + String(iPID, HEX));
        
        int result = obdRead(PID);

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

int obdRead(String PID) {
    char iPID = parsePID(PID); // Use the PID to know what function to use for parsing
    char* response;
    obdSerial.println(PID);
    //Serial.print(F("Reading ")); Serial.println((int)iPID);

    response = getResponse();
    delay(1000);
    response = getResponse();
    delay(200);

    if (parseFuncs[iPID] != NULL) {
        parseFunc pF = pgm_read_byte_near(parseFuncs + iPID);
        return pF(response);
    } else {
        return 0;
    }
}

char* getResponse(void) {
    static char response[20];
    char inChar = 0;
    char index = 0;

    // Keep reading chars from OBD until receives a carriage return (\r)
    while (inChar != '\r') {
        if (obdSerial.available() > 0) { // There is something to be read on the Serial port
            inChar = obdSerial.read();
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

/*
 *  Parse Formulas from
 *  https://en.m.wikipedia.org/wiki/OBD-II_PIDs
 */

int parseMIL(char* response) {
    char A = response[0];

    return A & (1 << 7);
}

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
    return (256*response[0] + response[1])/4;
}

int parseSpeed(char* response) {
    return response[0];
}

int parseAirFlowRate(char* response) {
    return (256*response[0] + response[1])/100;
}

int parseSeconds(char* response) {
    return 256*response[0] + response[1];
}

int parseDistance(char* response) {
    return 256*response[0] + response[1];
}

int parseMinutes(char* response) {
    return 256*response[0] + response[1];
}

int parseCount(char* response) {
  return response[0];
}

char* parseHex(char* data) {
    char tmp[3] = {' ', ' ', '\0'};
    static unsigned char response[4];
    char i, j;
    
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

void clearOBDSerial(void) {
    while (obdSerial.available() > 0) {
        obdSerial.read();
    }
}

/*
 *  Package Functions
 *  
 *  "msgId" : {
 *      "datestamp": "YYYY-MM-DD",
 *      "hourstamp": "HH:MM:SS+03:00",
 *      "pids": {
 *          "pid": "val",
 *          "pid": "val"
 *      },
 *      "placa": "AAA-0000"
 *  }
 */

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

void saveTimestamp() {
    // Timestamp format: YYYY-MM-DDTHH:MM:SS+03:00
    Serial.println(F("saving timestamp"));
    
    if (package) {
        package.print(F("\t\"timestamp\": \""));
        
        int year;
        byte month, day, hour, minutes, second, hundredths;
        gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths);
        
        char timestamp[25];
        sprintf(timestamp, "%02d-%02d-%02dT%02d:%02d:%02d-03:00", year, month, day, hour, minutes, second);
        
        package.print(timestamp);
        package.println("\",");
    }
}

void savePosition() {
    Serial.println(F("saving position"));
  
    if (package) {
        package.println(F("\t\"coord\": {"));
        package.print(F("\t\t\"lat\": ")); package.print(latitude, 6); package.println(F(","));
        package.print(F("\t\t\"long\": ")); package.print(longitude, 6); package.println(F(","));
        package.println(F("\t},"));
    }
}

void saveDistanceTravelled() {
    Serial.println(F("saving distance travelled"));
  
    if (package) {
        package.print(F("\t\"distanceTravelled\": "));
        package.print(distanceSinceLastMessage, 6); package.println(F(","));
    }
}

void startPIDs() {
    Serial.println(F("starting PID section"));

    if (package) {
        package.println(F("\t\"pids\": {"));
    }
}

void savePID(String PID, int val) {
    Serial.println(F("saving PID"));
    
    if (package) {
        package.print(F("\t\t\"")); package.print(PID);package.print(F("\": "));
        package.print(val);
        package.println(F(","));
    }
}

void endPIDs() {
    Serial.println(F("ending PID section"));
    
    if (package) {
        package.println(F("\t},"));
    }
}

void endPackage() {
    Serial.println(F("ending package"));
    
    if (package) {
        package.println(F("\t\"placa\": \"PCU-3455\""));
        package.println(F("}"));

        package.close();
    }
}

/*
void readPackage() {
    package = SD.open(F("package"));

    if (package) {
        Serial.println(F("Contents of 'package':"));

        while (package.available()) {
            Serial.write(package.read());
        }

        package.close();
    }
}


void readPackageInto(char* packageStr) {
    int packageStrSize = 1;
    int packageStrPosition = 0;
    packageStr = (char*)malloc(sizeof(char));
  
    package = SD.open("package");

    if (package) {
        while (package.available()) {
            if (packageStrPosition == packageStrSize) {
                packageStrSize += 10;
                packageStr = (char*)realloc(packageStr, packageStrSize*sizeof(char));
            }

            packageStr[packageStrPosition++] = package.read();
        }

        if (packageStrPosition == packageStrSize) {
            packageStrSize++;
            packageStr = (char*)realloc(packageStr, packageStrSize*sizeof(char));
        } else if (packageStrPosition < packageStrSize) {
            packageStr = (char*)realloc(packageStr, (packageStrPosition+2)*sizeof(char));          
        }
        packageStr[packageStrPosition++] = 0;

        package.close();
    }
}

void clearPackage() {
    SD.remove(F("package"));

    package = SD.open(F("package"), FILE_WRITE);
    package.close();
}

void saveToHistory(char* package) {
    history = SD.open(F("history"), FILE_WRITE);

    if (history) {
        history.println(package);

        history.close();
    }
}

void sendPackage() {
    //Serial.println(F("Sending package..."));
    char* package;

    readPackageInto(package);
    saveToHistory(package);

    clearPackage();

    // Send it via Tellit
    
    free(package);
    distanceSinceLastMessage = 0;
}*/
