/* This file was automatically generated.  Do not edit! */
char *getResponse(void);
char *parseHex(char *data);
float distanceBetweenCoordinates(float latitude1, float longitude1, float latitude2, float longitude2);
int main();
int16_t parseAirFlowRate(char *response);
int16_t parseCount(char *response);
int16_t parseDistance(char *response);
int16_t parseMIL(char *response);
int16_t parseMinutes(char *response);
int16_t parsePercentage(char *response);
int16_t parsePressure255(char *response);
int16_t parsePressure765(char *response);
int16_t parseRPM(char *response);
int16_t parseSeconds(char *response);
int16_t parseSpeed(char *response);
int16_t parseTemperature(char *response);
void clearOBDSerial(void);
void getAllowedPIDs();
void GPSManager()
void OBDManager();