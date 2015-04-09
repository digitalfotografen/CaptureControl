/*********************

Example code for the Adafruit RGB Character LCD Shield and Library

This code displays text on the shield, and also reads the buttons on the keypad.
When a button is pressed, the backlight changes color.

**********************/

// include the library code:
#include <Wire.h>
#include <Time.h>
#include <Camera.h>
#include <Metro.h>
#include <Statistic.h>

#include <SoftwareSerial.h> // we don use this, but it is used by Adafruit_GPS.h
#include <Adafruit_GPS.h>

// Use pin 3 and 4 for encoder input, interupt 1
// Use pin 10-13 for camera trigg

#define WHEEL_A 3
#define WHEEL_B 4

#define SHUTTER1 7
#define FOCUS1 6

#define SHUTTER_TIME 50 // ms
#define FOCUS_RATIO 80 // perceny av shutter period to use as focus period

#define FLASH_SENSOR 2 // flash sensor connected to pin 2 and uses interupt 0
#define FLASH_INTERUPT true
unsigned long lastFlash = 0;
int flashErrCount = 0;

// Using hardware serial (e.g. Arduino Mega) TX1 and RX1
Adafruit_GPS GPS(&Serial1);
// Did we have a GPS-fix before last update
boolean gpsFix = false;
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
#define GPS_BAUDRATE 4800
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
char lastNMEA[120] = "";


long distance = 0; // keeps track of position [mm]
boolean reverse = false; // direction of movement
long maxSpeed = 0;
long currentSpeed = 0;
long lastCounter = 0; // used to calculate speed
unsigned long lastTime = 0;
Statistic speedStats;
#define minAllowedSpeed 0
#define maxAllowedSpeed 2000

Metro speedMetro = Metro(100); // speed update interval, ms
Metro updateMetro = Metro(250); // display update interval, ms
Metro gpsMetro = Metro(10); // gps update interval, ms
Metro setTimeMetro = Metro(240000); // set time interval, ms

Camera camera = Camera(SHUTTER1, FOCUS1);

long defaultCapturePeriod = 300;
long camCounter = 0;
long capturePeriod = 0;
long focusPeriod = 0;

boolean hold = false;


void setup() {
  // Debugging output
  Serial.begin(115200);
  Serial.println("Setup...");
  
  // setup camera
  Serial.println("Initialize camera");
  Serial2.begin(4800); // setup camera serial port
  initCounters();
  camera.setExposureTime(SHUTTER_TIME);

  //setup GPS
  gpsSetup();
  
  Serial.setTimeout(250);

  //setup distance wheel sensor
  pinMode(WHEEL_A, INPUT);
  pinMode(WHEEL_B, INPUT);
  attachInterrupt(1, count, RISING);
  speedStats.clear();
  
  // setup flash sensor
  pinMode(FLASH_SENSOR, INPUT);
  attachInterrupt(0, interuptFlashSensor, FALLING);
 
  Serial.println("Setup finished!");
  // initialize connection with Processiong application
  Serial.println("&CONNECT");

  char buff[255] = "";
  csvString(buff, now(),true);
  Serial.print("&LOG="); Serial.println(buff);
}

void loop() {
  checkCamera(false);
  
  if (speedMetro.check()){
    updateSpeed();
  }
  

  if (gpsMetro.check()){
    gpsUpdate();
    if (gpsFix != GPS.fix){
      gpsFix = GPS.fix;
      if (gpsFix){
        Serial.println("GPS FIX OK!");
      } else {
        Serial.println("GPS lost satelites");
      }
    }
  }
  
  if (updateMetro.check()){
    updatePanel();
    //Serial.println(distance);

    if(Serial.available()){
      if (Serial.peek() == '&'){
        readSerial();
      } else {
        Serial.read();
        updatePanel();
        
      }
    }
    //setExifComment("0123456778902komment");
     
  }
}

void initCounters(){
  camCounter = 0;
  capturePeriod = defaultCapturePeriod;
  focusPeriod = 100 * capturePeriod / FOCUS_RATIO;
  camera.reset();

  distance = 0; // keeps track of position
  maxSpeed = 0;
  currentSpeed = 0;
  lastCounter = 0; // used to calculate speed
  lastTime = 0;
  speedStats.clear();
}

// Increment counters
void count(){
  if (!reverse){
    if (digitalRead(WHEEL_B)){
      distance--;
      camCounter--;
    } else {
      distance++;
      camCounter++;
    }
  } else {
    if (digitalRead(WHEEL_B)){
      distance++;
      camCounter++;
    } else {
      distance--;
      camCounter--;
    }
  }    
}


void updatePanel(){
  char str[255] = "";
  char buff[500] = "[{";
  
  sprintf(str, "camcount:%d", camera.counter());
  strcat(buff, str);
  
  strcat(buff, ",distance:");
  dtostrf(distance / 1000.0, 0, 2, str);
  strcat(buff, str);

  strcat(buff, ",speed:");
  dtostrf(currentSpeed / 1000.0, 0, 2, str);
  strcat(buff, str);
  
  strcat(buff, ",latitude:");
  dtostrf(GPS.latitude/100, 9, 6, str);
  strcat(buff, str);
  strcat(buff, ",longitude:");
  dtostrf(GPS.longitude/100, 9, 6, str);
  strcat(buff, str);
    
  strcat(buff, ",datetime:\"");
  dateTimeString(str, now());
  strcat(buff, str);
  strcat(buff, "\"");
  
  sprintf(str, ",period:%d", capturePeriod);
  strcat(buff, str);

  sprintf(str, ",hold:%d", hold);
  strcat(buff, str);

  sprintf(str, ",reverse:%d", reverse);
  strcat(buff, str);

  strcat(buff, "}]");
  Serial.println(buff);
}

/*
* Speed in mm/s
*/
void updateSpeed(){
   long t = millis() - lastTime;
   lastTime = millis();

   currentSpeed = 1000L * (distance - lastCounter) / t;
   lastCounter = distance;
   speedStats.add(abs(currentSpeed));
}

boolean speedOk(){
  if (abs(currentSpeed) < minAllowedSpeed)
    return false;
  if (abs(currentSpeed) > maxAllowedSpeed)
    return false;
  return true;
}

/// GPS functions

void gpsSetup(){
 
  // Switching to lower baudrate
  /*
  GPS.begin(57600);
  GPS.sendCommand(PMTK_SET_BAUD_9600);
  delay(1000);
  */
   
  // Switching to higher baudrate
  /*
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_57600);
  delay(1000);
  */
  
  // Switching to 4800 baud
  /*
  /GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_4800);
  delay(1000);
  */
  GPS.begin(GPS_BAUDRATE);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // send RMC & GGA messages
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // all data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  
  delay(1000);
}

boolean gpsUpdate(){
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    strcpy(lastNMEA, GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    
    if (!GPS.parse(lastNMEA))   
      return false;  // we can fail to parse a sentence in which case we should just wait for another

    // check if gps fix state changed
    if (GPS.fix != gpsFix){
      if (GPS.fix){
        Serial.println("GPS-fix OK");
        setTimeFromGPS();
      } else {
        Serial.println("WARNING GPS-fix LOST");
      }
      gpsFix = GPS.fix;
    }

    if (!GPS.fix) {
        //Serial.print("No Fix");
        return false;
    }

    if (setTimeMetro.check()){
        setTimeFromGPS();
    }
    
    // send GPS-data to camera
    Serial2.write(lastNMEA);
    Serial2.write("\n");
    delay(100);
    //Serial.println(lastNMEA);
  }
  return true;
}

void printGPSdata(){
  char buff[255] = "";
  
  Serial.print("Date: ");
  gpsDateTime(buff);
  Serial.println(buff);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", "); 
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  }
  
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// set time from GPS-time
void setTimeFromGPS(){
  setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year); 
}

// Creates as formated c-string of last GPS date and time, 2014-14-01 09:10:11
void gpsDateTime(char *buff){
  strcpy(buff, "");
  sprintf(buff, "20%02d-%02d-%02d %02d:%02d:%02d.%03d", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);
}

// Creates as formated c-string of date and time, 2014-14-01 09:10:11
void dateTimeString(char *buff, time_t datetime){
  strcpy(buff, "");
  sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d", 
                  year(datetime), 
                  month(datetime), 
                  day(datetime), 
                  hour(datetime), 
                  minute(datetime), 
                  second(datetime));
}


// create comma separated string for lig file, this is sent to Processing application
void csvString(char *buff, time_t datetime, boolean header){
  char str[64] = "";
  
  strcpy (buff, "");
  
  if (header){
    strcat(buff, "counter,datetime,distance,speed,latitude,longitude");
  } else {
    sprintf(str, "%0004d,", camera.counter());
    strcat(buff, str);
    
    dateTimeString(str, datetime);
    strcat(buff, str);
    strcat(buff, ",");

    dtostrf(distance / 1000.0, 0, 2, str);
    strcat(buff, str);
    strcat(buff, ",");

    dtostrf(currentSpeed / 1000.0, 0, 2, str);
    strcat(buff, str);
    strcat(buff, ",");  
    
    dtostrf(GPS.latitude/100, 9, 6, str);
    strcat(buff, str);
    strcat(buff, ",");
    dtostrf(GPS.longitude/100, 9, 6, str);
    strcat(buff, str);

  }

  strcat(buff, "\r\n");
}

// Test to set EXIF-comments in camera
// Work in progress, Currently this does not work
void setExifComment(char *buff){
  Serial2.write('$');
  Serial2.write(0x9286); //EXIF comment
  Serial2.write(0x0018); //datatype undefined
  uint8_t stringsize = strlen(buff);
  
  Serial2.write((uint8_t *)buff, stringsize); //datatype undefined
  Serial2.write("\r\n");
}

boolean checkCamera(boolean force){
  boolean result = false;
  if ((!hold && speedOk()) || force){
    if (abs(camCounter) >= focusPeriod){      
      camera.focus();
    }
    if ((abs(camCounter) >= capturePeriod) || force){      
      // get capture time here
      time_t time = now();
      camera.capture();
      camCounter = 0;
      char buff[255] = "";
      csvString(buff, time, false);
      Serial.print("&LOG="); Serial.print(buff);
      result = true;
    }
  }
  if (camera.checkTimer()){ // true if shutter just released
    if (millis() - lastFlash > 200){
      flashErrCount++;
      Serial.print("NO FLASH!!!! ");
      Serial.print(camera.counter());
      Serial.print("-");
      Serial.println(flashErrCount);
    }
  }
  return result;
}

// Read and parse serial commaneds from Processing application
void readSerial(){
  char buffer[255] = "";
  Serial.readBytesUntil('\n', buffer, 255);
  //Serial.println(buffer); //echo

  if(strncmp(buffer, "&CAPTURE", 8) == 0){
    checkCamera(true);
    return;
  }

  if(strncmp(buffer, "&SET", 4) == 0){
    char *name;
    name = strchr(buffer, '=') + 1;
    char *value;
    value = strchr(buffer, ':') + 1;
  
    if (strncmp(name, "distance", 8) == 0){
      distance = (long) 1000.0*atof(value);
    }
    if (strncmp(name, "period", 6) == 0){
      capturePeriod = atoi(value);
    }
    if (strncmp(name, "hold", 4) == 0){
      hold = atoi(value) != 0 ? true : false; 
    }
    if (strncmp(name, "reverse", 7) == 0){
      reverse = atoi(value) != 0 ? true : false; 
    }
    return;
  }
}

void interuptFlashSensor(){
  lastFlash = millis();
  //wSerial.println("FLASH DETECTED");
}
