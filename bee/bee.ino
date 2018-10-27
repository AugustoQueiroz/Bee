#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

/*
 *  GPS Initialiations
 */

const uint32_t GPSBaud = 9600;

TinyGPS gps;
static const int RXPin = 4, TXPin = 3;
SoftwareSerial softSerial(RXPin, TXPin);

float distanceSinceLastMessage = 0;
float currentLatitude = TinyGPS::GPS_INVALID_F_ANGLE;
float currentLongitude = TinyGPS::GPS_INVALID_F_ANGLE;

/*
 *  OBD Initializations
 */

/*
 *  Package Initializations
 */

File history;
File package;

int packageBuilderState = 0;

#define SEND_INTERVAL 10 // Interval between messages in seconds

void setup() {
    // put your setup code here, to run once:
    softSerial.begin(GPSBaud); // Serial for the GPS reader

    Serial.begin(38400); // Serial for the OBD reader

    if (!SD.begin(8)) {
        Serial.println("SD initialization failed.");

        while (1);
    }   

    SD.remove("package");
    SD.remove("history");

    package = SD.open("package", FILE_WRITE);
    package.close();

    history = SD.open("history", FILE_WRITE);
    history.close();
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
    static double lastLatitude = 0;
    static double lastLongitude = 0;

    while (softSerial.available() > 0) {
        if (gps.encode(softSerial.read())) {
            float latitude, longitude;
            
            gps.f_get_position(&latitude, &longitude);
            
            Serial.print("lat: "); Serial.print(latitude, 6); Serial.print(" lng: "); Serial.println(longitude, 6);

            if (lastLatitude != 0) {
                distanceSinceLastMessage += distanceBetweenCoordinates(latitude, longitude, lastLatitude, lastLongitude);
            }
            lastLatitude = latitude;
            lastLongitude = longitude;
        }
    }
}

void GPSDelay(unsigned long ms) {
  unsigned long start = millis();

  while (millis() - start < ms) {
    if (softSerial.available() > 0) gps.encode(softSerial.read());
  }
}

double distanceBetweenCoordinates(double latitude1, double longitude1, double latitude2, double longitude2) {
    double R = 6371e3;
    double phi1 = toRadians(latitude1);
    double phi2 = toRadians(latitude2);
    double deltaPhi = toRadians(latitude2 - latitude1);
    double deltaLambda = toRadians(longitude2 - longitude1);

    double a = sin(deltaPhi/2) * sin(deltaPhi/2) + cos(phi1)*cos(phi2)*sin(deltaLambda/2)*sin(deltaLambda/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));

    return R*c;
}

double toRadians(double angle) {
    return angle*PI/180;
}

/*
 *  OBD Functions
 */

typedef int (*parseFunc)(char*);

int allowedPIDs[13] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

parseFunc parseFuncs[] = { 
    NULL, 
    parseMIL, 
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
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);
    allowedPIDs[0] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[1] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("0120");
    response = getResponse();
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);
    allowedPIDs[2] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[3] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("0140");
    response = getResponse();
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);
    allowedPIDs[4] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[5] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("0160");
    response = getResponse();
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);
    allowedPIDs[6] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[7] = ((int) response[2] << 8) | (int) response[3];
    
    Serial.println("0180");
    response = getResponse();
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);
    allowedPIDs[8] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[9] = ((int) response[2] << 8) | (int) response[3];

    Serial.println("01A0");
    response = getResponse();
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);
    allowedPIDs[10] = ((int) response[0] << 8) | (int) response[1];
    allowedPIDs[11] = ((int) response[2] << 8) | (int) response[3];
}

void OBDManager() {
    static unsigned char iPID = 0;

    Serial.println(iPID);

    if (allowedPIDs[iPID/16] & (1 << (16 - iPID%16))) {
        String PID = String(String("01") + String((iPID < 16) ? "0" : "") + String(iPID, HEX));
        
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
            Serial.println("Starting to save PIDs/PIDs Saved");
        }
    }
}

int obdRead(String PID) {
    char iPID = parsePID(PID); // Use the PID to know what function to use for parsing
    char* response;
    //Serial.println(PID);

    /*response = getResponse();
    GPSDelay(1000);
    response = getResponse();
    GPSDelay(200);*/

    if (false && iPID < 92 && parseFuncs[iPID] != NULL) {
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

    if (millis() - lastReset > SEND_INTERVAL * 1000) {
        Serial.println("now yes");
        switch (packageBuilderState) {
        case 0:
            startPackage();
            saveTimestamp();
            savePosition();
            saveDistanceTravelled();
            startPIDs();
            packageBuilderState++;
            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            endPIDs();
            endPackage();
            sendPackage();
            lastReset = millis();
            packageBuilderState = 0;
            break;
        }
    } else {
        Serial.println("not yet");
    }
}

void startPackage() {
    package = SD.open("package", FILE_WRITE);

    if (package) {
        package.println("\"msg\" : {");

        package.close();
    } else {
      Serial.println("Could not open 'package' to start package.");
    }
}

void saveTimestamp() {
    // Timestamp format: YYYY-MM-DDTHH:MM:SS+03:00
    package = SD.open("package", FILE_WRITE);
    
    if (package) {
        package.print("\t\"timestamp\": \"");
        char timestamp[] = "YYYY-MM-DDTHH:MM:SS-03:00";
        int year;
        byte month, day, hour, minutes, second, hundredths;
        gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths);
        sprintf(timestamp, "%02d-%02d-%02dT%02d:%02d:%02d-03:00", year, month, day, hour, minutes, second);
        package.print(timestamp);
        package.println("\",");

        package.close();
    } else {
      Serial.println("Could not open 'package' to save TimeStamp.");
    }
}

void savePosition() {
    package = SD.open("package", FILE_WRITE);
    
    if (package) {
        package.println("\t\"coord\": {");
        //package.print("\t\t\"lat\": "); package.print(gps.location.lat(), 6); package.println(",");
        //package.print("\t\t\"long\": "); package.print(gps.location.lng(), 6); package.println(",");
        package.println("\t},");

        package.close();
    }
}

void saveDistanceTravelled() {
    package = SD.open("package", FILE_WRITE);

    if (package) {
        package.print("\t\"distanceTravelled\": ");
        package.print(distanceSinceLastMessage, 6); package.println(",");

        package.close();
    }
}

void startPIDs() {
    package = SD.open("package", FILE_WRITE);
    
    if (package) {
        package.println("\t\"pids\": {");

        package.close();
    } else {
      Serial.println("Could not open 'package' to start PIDs.");
    }
}

void savePID(String PID, int val) {
    package = SD.open("package", FILE_WRITE);

    if (package) {
        package.print("\t\t\"" + PID + "\": ");
        package.print(val);
        package.println(",");

        package.close();
    } else {
        Serial.println("Could not open 'package' to save PID.");
    }
}

void endPIDs() {
    package = SD.open("package", FILE_WRITE);
    
    if (package) {
        package.println("\t},");

        package.close();
    } else {
      Serial.println("Could not open 'package' to end PIDs.");
    }
}

void endPackage() {
    package = SD.open("package", FILE_WRITE);
    
    if (package) {
        package.println("\t\"placa\": \"PCU-3455\"");
        package.println("}");

        package.close();
    } else {
      Serial.println("Could not open 'package' to end it.");
    }
}

void readPackage() {
    package = SD.open("package");

    if (package) {
        Serial.println("Contents of 'package':");

        while (package.available()) {
            Serial.write(package.read());
        }

        package.close();
    } else {
      Serial.println("Could not open 'package' to read.");
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
    SD.remove("package");

    package = SD.open("package", FILE_WRITE);
    package.close();
}

void saveToHistory(char* package) {
    history = SD.open("history", FILE_WRITE);

    if (history) {
        history.println(package);

        history.close();
    }
}

void sendPackage() {
    Serial.println("Sending package...");

    SD.remove("package");
    char* package;

    readPackageInto(package);
    saveToHistory(package);

    // Send it via Tellit
    
    free(package);
    distanceSinceLastMessage = 0;
}
