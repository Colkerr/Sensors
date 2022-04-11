//have to define user and password
//need GPI16 joined to REST to awake from sleep
//Makes up, reads DHTxx, saves or transmits the data. 
//On boot reads littleFS OTA.txt to determine whether to connect to WiFi for OTA update, or open AP 192.168.4.1 for user input
//User can download data or use 192.168.4.1/extra to change defaults.
//Alternatively 192.168.4.1/input4OTA for inputting WiFi credentials and switching next boot to OTA mode

#define Sensor_User "******"           //<<<<<<<<<<<<<<<<<<< NOTE change these and possibly others below <<<<<<<<<<
#define Sensor_Password  "******"         //>>>> WARNING password may have to be at least 8 characters or WiFI.softAP(..) fails.
#define transmitPIN 12 
#define DHTpin 13 
             
#include <RCSwitch.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include "ESPFtpServer.h"
#include "LittleFS.h"
#include <EEPROM.h>
//#include "ESP8266FtpServer.h"
#include "DHT.h"
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

//unsigned int uiDelay, uiProtocol, uiLength; 
//RCSwitch mySendSwitch = RCSwitch();

extern "C" {          // this is for the RTC memory read/write functions
  #include "user_interface.h" 
}
#define RTCMEMORYSTART 65
typedef struct {      // this is for the RTC memory read/write functions
  int fixTime ;       //units of 0.1% that is milliseconds
  int iYr;            //4 bytes
  int iMnth ;    
  int iDay ;      
  int iHr;      
  int iMin;       
  int iSec;
  int iFileNum;
  int timings[2] = {1, 0};     //Cycle time in mins, spare
  int iSave2;  //0,1 for flash, radio to hub
  int iTimeSet;   //0 for no records 1 to record anyway
  int iDhtType;   //1 or 2 for DHT11 or 22
} rtcSubStore;

typedef struct {
  uint32_t crc32;
  rtcSubStore rtcData;
} rtcStore;

rtcStore rtcMem;

long waitForSetUp = 1 * 60 * 1000;          // time out wait for setup 
long extendWaitForSetUp = 4 * 60 * 1000;   // once setup started extend to 4 minutes
unsigned long ulDHTbegin;

ADC_MODE(ADC_VCC);   // enable voltage monitoring
DHT dht11(DHTpin, DHT11);
DHT dht22(DHTpin, DHT22);
RCSwitch mySwitch = RCSwitch();
ESP8266WebServer server(80);    // Create a webserver object that listens for HTTP request on port 80
FtpServer ftpSrv;

bool bOTAmode;
unsigned long previousMillis = 0;
String sLocation="", sSensors="", sFileDescriptor = "", sMAC ="", sInterval="", sMemory, sSensorType, sIfNoTimes ;   
int Temperature, Humidity, runTime;       // milliseconds in the process
float fTemperature, fHumidity;
byte MAC[6];
//String dhtTypes[] = {"DHT11", "DHT22"};
String save2Types[] = {"flash", "radio", "wifi"};
String sFileName, sFileDate, sFileTime,sOTA_PW, sOTA_SSID;

void setup() {
  Serial.begin(9600);  
  sFileName = __FILE__;
  sFileDate = __TIME__;
  sFileTime = __DATE__;
  Serial.println(sFileName); Serial.println(sFileDate); Serial.println(sFileTime); 
  //delay(3000); //for testing
  if (LittleFS.begin()) Serial.println("LittleFS begin OK");
  Serial.println("Initialise start up -> setup ");
  checkFileState();   //and initialise 
  if ( OTAmode() ) {
    bOTAmode = true;
    OTA_disable();  //so next time normal mode
    OTA_routine();
  } else {
    readFromRTCMemory();
    pinMode(DHTpin, INPUT);
    
    ulDHTbegin = millis();  //no need to wait longer than 2 secs since begin.
    uint32_t crc32Calc = calculateCRC32((uint8_t*) &rtcMem.rtcData, sizeof(rtcMem.rtcData));
    
    if (rtcMem.crc32  == crc32Calc ) { 
      //>>>>>>>>>>>>>READ SENSORS THEN SLEEP<<<<<<<<<<<<<  
      delay(2);               
      //transmitData(Temperature, Humidity);
      maintainClock();
      setSensorType();
      readSensors(true);   //and save data to LittleFS
      writeToRTCMemory();   //after maybe changing fNum
      delay(2);  
      ESP.deepSleep( 1e3*(60e3*rtcMem.rtcData.timings[0] + 600 - millis() ), WAKE_RF_DISABLED); //400 fine adjustment
      //ESP.deepSleep( 1 * 60e6  + 400 - millis() , WAKE_RF_DISABLED); 
    } else { 
      //>>>>>>>>>>>>>serve web page<<<<<<<<<<<<<  
      waitForSetUp = extendWaitForSetUp + millis(); //must be intended restart so allow longer
      setSensorType();
      delay(10);     
      WiFi.begin();
      WiFi.macAddress(MAC);
      sMAC = "";
      for (int i = 0; i < sizeof(MAC); i++) {Serial.print(MAC[i], HEX); sMAC = sMAC + String(MAC[i], HEX);}
      sMAC.toUpperCase();
      //WiFi.forceSleepWake();
      if (!WiFi.softAP(Sensor_User, Sensor_Password)) {Serial.println("false returned from WiFi.softAP() "); }             // Start the access point
      Serial.print("Access Point started IP adress = ");
      Serial.println(WiFi.softAPIP());               // Show IP address
      ftpSrv.begin(Sensor_User, Sensor_Password); // port 21
      server.on("/shareData", shareData);
      server.on("/saveExtraInputs", saveExtraInputs);
      server.on("/deleteAllRecords", deleteAllRecords);
      server.on("/extra", showExtraInputs);
      server.on("/input4OTA", input_OTA_creds);
      server.on("/prepare_OTA_boot", prepare_OTA_boot); 
      server.on("/", showSetup);
      server.onNotFound([]() {                              // If the client requests any URI
       // if (!handleFileRead(server.uri()))                  // send it if it exists
          server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
      });
      server.begin();                           
      Serial.println("HTTP server started");
      delay(10);
    } 
  }
}

void loop() {   //waiting for client then timeout and sleep
  if ( bOTAmode ) {
    ArduinoOTA.handle();
  } else {
    if ( millis() > waitForSetUp ) {
        Serial.println("End of wait time");            //start cycling
        rtcMem.rtcData.iYr=0;rtcMem.rtcData.iMnth=0;rtcMem.rtcData.iDay=0;rtcMem.rtcData.iHr=0;rtcMem.rtcData.iMin=0;rtcMem.rtcData.iSec=0;  //default when not set explicitly        
        readSensors(true);   //and save data to LittleFS
        writeToRTCMemory();
        WiFi.mode(WIFI_OFF);
        Serial.println(" Sleep until state 1");
        delay(10);
        ESP.deepSleep(100e6, WAKE_RF_DISABLED);   //sleep and wake in sensing mode
    } else {
      //ftpSrv.handleFTP(LittleFS);
      server.handleClient();
    }
    delay(10);
  }
}

void setSensorType() {
  if (rtcMem.rtcData.iDhtType == 1 ) {
    dht11.begin();
  } else { 
    dht22.begin();
  }
}

String makeDateTime() {
  return  sFormInt(rtcMem.rtcData.iYr,4,'0') + "-" + sFormInt(rtcMem.rtcData.iMnth, 2, '0') + "-" + sFormInt(rtcMem.rtcData.iDay, 2, '0') +
          " " + sFormInt(rtcMem.rtcData.iHr, 2, '0') + ":" + sFormInt(rtcMem.rtcData.iMin, 2, '0') ;
}

String readSensors(bool bSaveData) {   //and save to file  
  if (millis() - ulDHTbegin < 500) {
    Serial.print("dht warm up delay "); Serial.println(500 - (millis() - ulDHTbegin));
    delay(500 - (millis() - ulDHTbegin));  //how long to warm up ?
  }
  if (rtcMem.rtcData.iDhtType == 1 ) {
      fTemperature = dht11.readTemperature(); 
      fHumidity = dht11.readHumidity();
  } else {
      fTemperature = dht22.readTemperature(); 
      fHumidity = dht22.readHumidity();
  }

  if (fTemperature < -10 || fTemperature > 99 ) fTemperature = 0;  //in case spurious values returned
  if (fHumidity < 0 || fHumidity > 100 ) fHumidity = 0;
  String sSensors = String(fTemperature) + "," + String(fHumidity);
  int iTemp = fTemperature + 0.5;
  int iHumid = fHumidity + 0.5;
  sSensors = String(iTemp) + "," + String(iHumid);
  Serial.println(sSensors + "<");
  if (bSaveData) {
    if (rtcMem.rtcData.iSave2 == 1) {
      unsigned long ulData; //value to return
      long lTempP50_100;
      delay(10);
      mySwitch.setProtocol(6);
      mySwitch.setPulseLength(200); 
      mySwitch.enableTransmit(12);   //using pin 12
      lTempP50_100 = ( 40 + iTemp ) * 100;  //offset 40 to allow for below zero and shift x 100
      ulData = 12340000  + lTempP50_100 + iHumid; 
      transmit(ulData, 6, 24);  //test transmit      
    } else {
    saveRecord(String(makeDateTime()) + "," + sSensors);
    }
  }
  return(sSensors);     // for html if called from setDateTime()
}

void showSetup() {
  waitForSetUp = extendWaitForSetUp; //must be intended restart so allow longer  
  server.send(200, "text/html", "<h1>SETUP</h1>"
      "<form method='post' name='frm' action='/shareData'> "
      "<input type='hidden' name='product' size='45' /> <br>"
      "<br> The sensor will hibernate in 4 minutes if you do nothing."
      "<br> Please download the data every week or two and send it to data@heathack.org "
      "<br><br>"
      " This should show reasonable values for temperature,humidity: " + readSensors(false) +  " cound<br><br>"
      " Enter the location &nbsp; &nbsp; "       
         "<input type='text'  name='description' value = '" + sLocation + "' ><br><br>"
         "<p><input type='submit' value='DOWNLOAD DATA AND START' &nbsp;&nbsp; &nbsp; />"   
         "</form> <br> "     
      "<br><br>Battery voltage: &nbsp; &nbsp; " + getVoltage() + "v"
      "<br><br>Save to: &nbsp; &nbsp; " + sMemory + ""
      "<br><br>Interval: &nbsp; &nbsp; " + sInterval + ""
      "<br><br>When no date/time set: &nbsp; &nbsp; " + sIfNoTimes + ""
      "<br><br>Sensor Type: &nbsp; &nbsp; " + sSensorType + ""
      "<br><br>Version: &nbsp; &nbsp; " + sFileName + "   " + sFileDate + "  "  + sFileTime + "" 
      "<script>document.frm.product.value=Date(); </script>"
   );
}

String getVoltage() {
  float fBattVoltage = ESP.getVcc();
  String sBattVoltage = String(fBattVoltage/1000,2);
  return sBattVoltage;
}

void setDateTime(String sTimestamp) {
  rtcMem.rtcData.iMnth = month2Number(sTimestamp.substring(4, 7));
  rtcMem.rtcData.iYr = sTimestamp.substring(11, 15).toInt();
  rtcMem.rtcData.iDay = sTimestamp.substring(8, 10).toInt();
  rtcMem.rtcData.iHr = sTimestamp.substring(16, 18).toInt();
  rtcMem.rtcData.iMin = sTimestamp.substring(19, 21).toInt();
  rtcMem.rtcData.iSec = sTimestamp.substring(22, 24).toInt();
  //if ( rtcMem.rtcData.iSec > 59 ) { rtcMem.rtcData.iMin = rtcMem.rtcData.iSec/ 60; rtcMem.rtcData.iSec = rtcMem.rtcData.iSec % 60;}
  //if ( rtcMem.rtcData.iMin > 59 ) { rtcMem.rtcData.iHr = rtcMem.rtcData.iMin/ 60; rtcMem.rtcData.iMin = rtcMem.rtcData.iMin % 60;}
  //if ( rtcMem.rtcData.iHr > 24 ) { rtcMem.rtcData.iDay = rtcMem.rtcData.iHr/ 24; rtcMem.rtcData.iHr = rtcMem.rtcData.iHr % 24;}
  
  //readSensors in following statement will also save first reacord to file
  /*server.send(200, "text/html", "Date & Time is " + sFormInt(rtcMem.rtcData.iYr, 4, '0') + "/" + sFormInt(rtcMem.rtcData.iMnth, 2, '0')  + "/" + sFormInt(rtcMem.rtcData.iDay, 2, '0') + 
          " " + sFormInt(rtcMem.rtcData.iHr, 2, '0') + ":" + sFormInt(rtcMem.rtcData.iMin, 2, '0') + ":" + sFormInt(rtcMem.rtcData.iSec, 2, '0') +
          "<br>First record = " + readSensors() + "<br>"
          "<form  method='post' name='return' action='/'>"
          "<p><input type='submit' value='Return to Setup' />"
          "</form>"
         ); 
  */      
  delay(100);  
}

void maintainClock() {
  rtcMem.rtcData.iMin += rtcMem.rtcData.timings[0];  
    if (rtcMem.rtcData.iMin > 59) {
      rtcMem.rtcData.iMin = rtcMem.rtcData.iMin % 60;
      if (++rtcMem.rtcData.iHr > 23) {
        rtcMem.rtcData.iHr = 0;
        if (++rtcMem.rtcData.iDay > daysInMonth(rtcMem.rtcData.iYr,rtcMem.rtcData.iMnth,rtcMem.rtcData.iDay)) {
           rtcMem.rtcData.iDay = 1; ++rtcMem.rtcData.iMnth;
           if (rtcMem.rtcData.iMnth > 12) {
             ++rtcMem.rtcData.iYr; rtcMem.rtcData.iMnth = 1;
           }
        }
      }
  }
}

void deleteAllRecords() {
  Serial.println("handle delete");
  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    String dirFile = dir.fileName();
    File f = dir.openFile("r");
    Serial.print(" will delete file = "); Serial.print(dirFile);Serial.print(" size "); Serial.println(f.size());
    f.close();
    LittleFS.remove(dirFile);
  }  
  checkFileState();
  showSetup();
}

void xxdeleteAllRecords() {              
  server.send(200, "text/html", "<form  method='post' name='return' action='/'>"
          "<p><input type='submit' value='Confirm Delete' &nbsp;&nbsp; &nbsp; />"
          "</form>"
          "<form  method='post' name='return' action='/'>"
          "<p><input type='submit' value='Cancel' />"
          "</form>"
         );      
}

void shareData() {
  String sTimestamp= server.arg(0); //date time
  sLocation= server.arg(1);  //description
  saveLocation(sLocation);
  setDateTime(sTimestamp);
  unsigned long ulAdjTime= millis();
  int iNum = 1;
  //String sHead = "<!DOCTYPE html>";   //<html><body>";
  server.sendHeader("Content-Disposition", "attachment; filename='datalog.csv'");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain");    
 // server.sendContent(sHead ); 
  server.sendContent("$A,ReBoot@," + makeDateTime() + String("\n") ); 
  server.sendContent("$B," + sMAC + "," + getVoltage() + String("\n") ); 
  File dataFile = LittleFS.open("/SensorDataFile_1.csv", "r");
  while (dataFile) {
    Serial.print(" file num ");Serial.println(iNum);
    String dataString = dataFile.readStringUntil(EOF);     
    server.sendContent(dataString); 
    dataFile.close();
    dataFile = LittleFS.open("/SensorDataFile_" + String(++iNum) + ".csv", "r");
  }
  dataFile.close();
  server.sendContent("$Z,End of Records");
  server.sendContent("");     //marks the end of server message
  Serial.println("end fetch");

  rtcMem.rtcData.iSec = rtcMem.rtcData.iSec + (millis() - ulAdjTime)/1000 ;  
  if ( rtcMem.rtcData.iSec > 59 ) { rtcMem.rtcData.iMin++ ; rtcMem.rtcData.iSec = rtcMem.rtcData.iSec % 60;}
  if ( rtcMem.rtcData.iMin > 59 ) { rtcMem.rtcData.iHr++ ; rtcMem.rtcData.iMin = rtcMem.rtcData.iMin % 60;}
  if ( rtcMem.rtcData.iHr > 23 ) { rtcMem.rtcData.iDay++ ; rtcMem.rtcData.iHr = rtcMem.rtcData.iHr % 24;}
 
  saveRecord("$C," + sFileName + "  " + sFileDate + "  " + sFileTime);
  saveRecord("$D," + makeDateTime() + "," + sLocation );
  saveRecord("$E," + sInterval + "," + sMemory + "," + sSensorType + "," + sIfNoTimes);
  //saveRecord(sNewBoot);
  saveRecord("YYYY-MM-DD hh:mm,Temp,Humid");  //heading
  readSensors(true);  //updates lrecordNum
  writeToRTCMemory();
  yield();
  WiFi.mode(WIFI_OFF);
  unsigned long napTime = 1e3*(60e3*rtcMem.rtcData.timings[0] - ulAdjTime);
  if (napTime > 30*60e6 ) napTime = 1000;  //max sleep should be interval less time to here (1 minute + 4 minutes). Play safe use 30 minutes
  Serial.print(" naptime ");Serial.println(napTime);
  //ESP.deepSleep(napTime, WAKE_RF_DISABLED);   //sleep 
  //Serial.print(" sleep for ");Serial.println(1e3*(60e3*rtcMem.rtcData.timings[0] - millis());
  //delay(100);
  ESP.deepSleep(1e3*(60e3*rtcMem.rtcData.timings[0] - millis() ), WAKE_RF_DISABLED);   //sleep 
  //ESP.deepSleep(60e6, WAKE_RF_DISABLED);
} 

void saveRecord(String output)  {
  File f2;
  int iNum = rtcMem.rtcData.iFileNum;
  String sNum = String(iNum);
  f2 = LittleFS.open("/SensorDataFile_" + sNum + ".csv", "a");
  if (!f2) {
    Serial.println(" @A failed to open /SensorDataFile_" + sNum + ".csv");
  } else {
    Serial.println(" opened /SensorDataFile_" + sNum + ".csv");
  }
  int fsize = f2.size();
  if (fsize > 10000 ) {
    f2.close();
    rtcMem.rtcData.iFileNum = (rtcMem.rtcData.iFileNum > 20)? 1:(rtcMem.rtcData.iFileNum + 1);
    sNum = String(rtcMem.rtcData.iFileNum);
    File f1 = LittleFS.open("/SensorMetaFile.txt", "w");  
    f1.print(sNum);
    f1.close();
    LittleFS.remove("/SensorDataFile_" + sNum + ".csv");
    f2 = LittleFS.open("/SensorDataFile_" + sNum + ".csv", "a");
    Serial.print(" created new file : ");Serial.println(sNum);
  }
  Serial.print(" C file size : ");Serial.println(f2.size());
  f2.println(output);
  delay(10);
  f2.close();
  Serial.print(" output ");Serial.println(">" + output +"<");
  delay(10);
  yield();
}

//For FTP
bool handleFileRead(String path) { // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  File dataFile = LittleFS.open(path, "r");
  if (path == "SensorDataFile_1.csv") {
    int iF = 2;
    while (dataFile) {
      String dataString = dataFile.readStringUntil(EOF); 
      server.sendContent(dataString);
      dataFile.close();
      File dataFile = LittleFS.open("SensorDataFile_" + String(iF++) + ".csv", "r");
    } 
  } else {
    String dataString = dataFile.readStringUntil(EOF); 
    server.sendContent(dataString);
    dataFile.close();
  }
}

int month2Number(String sMonth) {
  String sMonths[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  int i = 0;
  while (sMonths[i] != sMonth) {
    i += 1;
  }
  return i + 1;
}

int daysInMonth(int iYr,int iMnth, int iDay) {
  int daysInMonths[12] = {31, 28, 31, 30, 31, 30, 31, 30, 31, 31, 30, 31};
  int numDays = daysInMonths[iMnth - 1];
  if (iMnth == 2)  {
    if (iYr % 4  == 0) {
      if (iYr % 100 != 0) {
        numDays = 29;
      } else {
        if (iYr % 400 == 0) {
          numDays = 29;
        }
      }
    }
  }
  return numDays;
}

void saveLocation(String sLocation) {
    File f1 = LittleFS.open("/Location.txt", "w");   
    f1.println(sLocation);
    f1.close();
}
void checkFileState() {   //only at startup or delete all files
  Serial.println(" in checkFileState()  "); 
  File f0, f1;
  if (!LittleFS.exists("/SensorMetaFile.txt" ) ) {
    Serial.println("no sensor meta file");
    if (LittleFS.format()) Serial.println("File System Formated");
    f1 = LittleFS.open("/SensorMetaFile.txt", "w");   
    f1.print("1");   //file number 1
    rtcMem.rtcData.iFileNum = 1;
    f1.close();

    f1 = LittleFS.open("/Location.txt", "w");   
    sLocation = "No location set yet";
    f1.println(sLocation);
    f1.close();
    
    sInterval = "5";
    sMemory = "Use Memory";   //default
    sSensorType = "DHT22";
    sIfNoTimes = "Hibernate";
    saveExtras();
    File f2 = LittleFS.open("/SensorDataFile_1.csv", "a");
    f2.println("YYYY-MM-DD hh:mm,Temp,Humid");  //heading
    f2.close();
  } else  {   
    restoreExtras();
    f1 = LittleFS.open("/SensorMetaFile.txt", "r");  
    String sFileNum = f1.readStringUntil('\0');
    f1.close();
    rtcMem.rtcData.iFileNum = sFileNum.toInt();
    
    f1 = LittleFS.open("/Location.txt", "r"); 
    sLocation = f1.readStringUntil('\0');
    f1.close();
  }
  f0.close();
}  

void restoreExtras() {
  File f0 = LittleFS.open("/extraSetup.txt", "r");
  sInterval = f0.readStringUntil('/');
  sMemory = f0.readStringUntil('/');
  sSensorType = f0.readStringUntil('/');
  sIfNoTimes = f0.readStringUntil('/');
  rtcMem.rtcData.timings[0] = sInterval.toInt();
  (sMemory == "Memory") ? (rtcMem.rtcData.iSave2 = 0): (rtcMem.rtcData.iSave2 = 1);
  (sSensorType == "DHT22") ? (rtcMem.rtcData.iDhtType = 2): (rtcMem.rtcData.iDhtType = 1);
  (sIfNoTimes == "Hibernate") ? (rtcMem.rtcData.iTimeSet = 0): (rtcMem.rtcData.iTimeSet = 1);
}
void saveExtraInputs() {
  sMemory = server.arg(0);
  sSensorType = server.arg(1);
  sInterval = server.arg(2);
  sIfNoTimes = server.arg(3);
  (sInterval == "Memory") ? (rtcMem.rtcData.iSave2 = 0): (rtcMem.rtcData.iSave2 = 1);
  (sSensorType == "DHT22") ? (rtcMem.rtcData.iDhtType = 2): (rtcMem.rtcData.iDhtType = 1);
  rtcMem.rtcData.timings[0] = server.arg(2).toInt();
  (sIfNoTimes == "Hibernate") ? (rtcMem.rtcData.iTimeSet = 0): (rtcMem.rtcData.iTimeSet = 1);
  saveExtras();
  setSensorType(); 
  writeToRTCMemory();
  showSetup();
}

void saveExtras() {
    File f0 = LittleFS.open("/extraSetup.txt", "w");
    f0.print(sInterval + "/");
    f0.print(sMemory + "/");
    f0.print(sSensorType + "/");
    f0.print(sIfNoTimes + "/");
}  

void readFromRTCMemory() {
  system_rtc_mem_read(RTCMEMORYSTART, &rtcMem, sizeof(rtcMem));
  yield();
}

void writeToRTCMemory() { 
  uint32_t crc32Calc = calculateCRC32((uint8_t*) &rtcMem.rtcData, sizeof(rtcMem.rtcData));
  rtcMem.crc32 = crc32Calc;
  system_rtc_mem_write(RTCMEMORYSTART, &rtcMem, sizeof(rtcMem));
  yield();
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

  
void showExtraInputs() {   //process
  String s5Checked = "", s10Checked = "",sSave2MemChecked="",sSave2HubChecked="";
  String  sHibernate = "", sKeepRecording = "", sDhtType11Checked = "", sDhtType22Checked = "";
  rtcMem.rtcData.timings[0] == 5 ? Serial.println("5 true"):Serial.println("5 false");
  rtcMem.rtcData.iDhtType == 1 ?  Serial.println("1 true"):Serial.println("1 false");
  rtcMem.rtcData.iTimeSet == 0 ?  Serial.println("0 true"):Serial.println("0 false");
  rtcMem.rtcData.iSave2 == 0 ?  Serial.println("0 true"):Serial.println("0 false");
  
  (rtcMem.rtcData.timings[0] == 5) ? (s5Checked = "checked"):(s10Checked = "checked");
  (rtcMem.rtcData.iDhtType == 1) ? (sDhtType11Checked = "checked"):(sDhtType22Checked = "checked");
  (rtcMem.rtcData.iTimeSet == 0) ? (sHibernate = "checked"):(sKeepRecording = "checked");
  (rtcMem.rtcData.iSave2 == 0) ? (sSave2MemChecked = "checked"):(sSave2HubChecked = "checked");

  server.send(200, "text/html", "<h1>Additional Inputs: </h1>"
      "<form method='post' name='frm' action='/saveExtraInputs'> "
      "1) Save to memory or transmit to hub.  <br>"       
            "<input type='radio' id='save2Mem' name='save2' value='Memory'"  + sSave2MemChecked + ">"
            "<label for='memory'>Memory</label><br>"
            "<input type='radio' id='save2Hub' name='save2' value='Hub'" + sSave2HubChecked + ">"
            "<label for='hub'>Send to Hub</label><br><br>"   
      "2) Sensor Type.  <br>"
            "<input type='radio' id='sensor' name='sensor' value='DHT22'" + sDhtType22Checked + ">"
            "<label for='dht22'>DHT22</label><br>"
            "<input type='radio' id='hub' name='sensor' value='DHT11'" + sDhtType11Checked + ">"
            "<label for='dht11'>DHT11</label><br><br>"
      "3) Minutes between sensor readings:<br>"
            "<input type='radio' id='5min' name='cycleTime' value='5'" + s5Checked + ">"
            "<label for='5Minutes'>5 Minutes</label><br>"
            "<input type='radio' id='10min' name='cycleTime' value='10'" + s10Checked + ">"
            "<label for='10Minutes'>10 Minutes</label><br><br>"   
      "4) If no Start Time has been set:<br>"
            "<input type='radio' id='continue' name='timeset' value='Continue'" + sKeepRecording + ">"
            "<label for='yesRecord'>Record anyway</label><br>"
            "<input type='radio' id='hibernate' name='timeset' value='Hibernate'" + sHibernate + ">"
            "<label for='notRecord'>No recording</label><br><br>"
          "<div><button type='submit'>Submit</button></div>"    
      
         "</form>"
   );
  Serial.print("@B rtcMem.rtcData.iTimeSet ");Serial.println(rtcMem.rtcData.iTimeSet);
 }

void transmit(unsigned long data, int repeats, int bitLen) { //###
  delay(5);
  mySwitch.setRepeatTransmit(repeats);
  mySwitch.send(data, bitLen); 
  //Serial.print(" transmitted data ");Serial.print(data); Serial.print("  bitlen  ");Serial.println(bitLen);
  //delay(10);    //##1 reduced from 500
}

//////////////////////Format integer
String sFormInt(int n, int i, char sP) {  //n to be formatted as i digits, leading sP
  String sN = String(n);
  int j = i - sN.length();
  for (int k = 0; k < j; k++)  {
    sN = sP + sN;
  }
  return sN;
}

void input_OTA_creds() {
   server.send(200, "text/html", "<h1>Additional Inputs: </h1>"
      "<form method='post' name='frm' action='/prepare_OTA_boot'> "
       " Input SSID and PASSWORD &nbsp; &nbsp; "       
         "<input type='text'  name='ssid' value = '            ' ><br><br>"
         "<input type='text'  name='pw'   value = '            ' ><br><br>"
         "<p><input type='submit' value='OTA on reboot' &nbsp;&nbsp; &nbsp; />"   
         "</form> <br> "
   );
}


bool OTAmode() {
  Serial.println(" checking OTA mode ");
  if (!LittleFS.exists("/OTA.txt" ) ) {
    return false;
  } else {
    File f0 = LittleFS.open("/OTA.txt","r") ;
    if ( f0.size() < 5 ) return false; 
    sOTA_SSID = f0.readStringUntil('/');
    sOTA_PW = f0.readStringUntil('/');
    WiFi.begin(sOTA_SSID, sOTA_PW);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println(" Could not connect to WiFi for OTA ");
      return false;
    }
    return true;
  }
}

void OTA_disable() {
  File f0 = LittleFS.open("/OTA.txt", "w");  
  f0.print("");
  f0.close();
}

void prepare_OTA_boot() {
  sOTA_SSID = server.arg(0);
  sOTA_PW = server.arg(1);
  File f0 = LittleFS.open("/OTA.txt", "w");  
  f0.print(sOTA_SSID + "/" + sOTA_PW + "/");
  f0.close();
  delay(2000);
  ESP.restart();
}


//OTA stuff
void OTA_routine() {
  Serial.println(" inside OTA stuff" );
  delay(2000);
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
