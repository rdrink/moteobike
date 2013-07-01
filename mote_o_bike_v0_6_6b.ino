/* --- mote-o-bike --- */
/* This is code for mote-o-bike, an Arduino based bike-mounted data collector
 /  Code is from a variety of authors & sources, including Robb Drinkwater,
 /  Douglas Pancoast, and students at the School of the Art Institute of Chicago,
 /  all of which was & is offered free & open-source under various licenses
 /
 /  An Adruino of sufficient size and power is assumed (Duemilanove or better),
 /  and the following shields are connected:
 /  - Sparkfun or Adafruit SD Datalogger
 /  - Sparkfun GPS Shield with GPS module of choice
 /
 /  The code below further assumes you have either the mote-o-bike Shield (fortcomming) 
 /  or the following wired up:
 /  - LEDs on pins 8 & 9 (not required, status only)
 /  - a Sensirion SHT-15 sensor on pins 4 & 5 (note library below)
 /  - a Sharp Light sensor (from Modern Device) on Analog pin 0
 /  - a MQ-35 Air Quality sensor (from Futerlec) read on Analog pin 1, run heater on Analog pin 6 as PWM
 / 
 /  Versions:
 /  v0.5
 /  - Added, then removed, battery level
 /  v0.6b (beta)
 /  - Added DEBUG_SENSOR
 /  - moved analogWrite to turn on heater to the end of setup
 /  v0.6.1b
 /  - changed datestamp() to MySQL YYYY-MM-DD format
 / v0.6.2b
 /  - changed heater level to a variable
 /  - made sample time (gps_interval) a variable and reduced to 3 secs. 
 / v0.6.3b
 / - changed serial speed to 57600 to be compatible with PySerial script
 / - added VESRSION so (eventually) website/app can tell the user if there is new software
 / - created 'breathing' startup
 / * recent r.d. edits *
 / - added infinite 'while' after dump to stop going back into logging
 / - removed GMT offset
 / v0.6.4b
 / - this version uses the <DTH.h> library instead of the SHT library
 / v0.6.5b
 / - this version uses the DHT22 for temp/hum
 / - added routine to get ID from eeprom (see sketch IDmote for details)
 / v0.6.6b
 / - 
 */


#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <EEPROM.h>
#include <DHT22.h> // include the DHT library
//#include "Wire.h"
//#include <LibTempTMP421.h>


Sd2Card card;
SdVolume volume;
SdFile root;

// Debugging
#define DEBUG // uncomment to turn debugging on
#define DEBUG_SENSOR // uncomment to see sensor values

#define BLINK 
#define RED_LED 8
#define GRN_LED 9
//#define BLU_LED 9;

String VERSION = "066b"; // change accoringly with each update
String DEVICE_ID = "MOTE-1"; // later we will get this out of the bluetooth module
int INTERVAL = 15; // GPS sample time in seconds

float TEMP_F, TEMP_C, HUMIDITY = -1.0; 
int AIR, LUX = -1;

const int chipSelect = 10; // 10 for Adafruit & Sparkfun shields
// offset = direction * longitude * 24 / 360; // dir = -1 for west, 1 for east

float latitude, longitude, altitude, course, mph;
char new_lat[6], new_lon[6];
unsigned long fix_age;



int dump_retries = 3;
int year;
int rand_num = 0; // temp rand_numom num
byte month, day, hour, minute, second, hundredths;
String gpsData, dateStamp, timeStamp, csvString = "";
boolean locked = false;
boolean card_ok = false;
int gps_interval = 5000; // time between GPS samples in mS

int heater_level_percent = 75; // run heater at 50%, 75%, etc.

int breath[] = {0,2,4,8,16,32,64,128,511,128,64,32,16,8,4,2,0}; // ramp for breathing effect

String RUN_NUMBER;
char file_name[] = "datalog.txt"; // default file name
File dataFile; // Global definition of file handler

// Define soft serial pins
#define RXPIN 2
#define TXPIN 3
// baud rate of the GPS module
#define GPSBAUD 4800

// Create an instance of the TinyGPS object
TinyGPS gps;

// This is where you declare prototypes for the functions that will be 
// using the TinyGPS library.
void getgps(TinyGPS &gps);

// Arduino SoftwareSerial (instead of NewSoftSerial)
SoftwareSerial uart_gps(RXPIN, TXPIN);

// temp & humidity  
// DHT sensor
#define DHT22_PIN 6
DHT22 dht(DHT22_PIN);
// TI TMP421 sensor
//LibTempTMP421 temp = LibTempTMP421(0);

//--- SETUP ---//
void setup()
{
  // Set up serial
  Serial.begin(57600); // chossen for best hardware-to-hardware communication
  Serial.flush();

#ifdef DEBUG
  Serial.println("Setup...");
#endif

  pinMode(A0, INPUT); // AMBI light sensor
  pinMode(A1, INPUT); // Air sensor

  pinMode(6, OUTPUT); // PWM for Air sensor 
  //pinMode(7, OUTPUT);
  pinMode(8, OUTPUT); // status LED
  pinMode(9, OUTPUT); // status LED

  // Setup for the logger
  pinMode(10, OUTPUT);       // Pin 10 must be set as an output for the SD communication to work.
  pinMode(11, OUTPUT);       // Pin 11 must be set as an output for the SD communication to work.
  pinMode(13, OUTPUT);       // Pin 13 must be set as an output for the SD communication to work.
  pinMode(12, INPUT);        // Pin 12 must be set as an input for the SD communication to work.

  // Start GPS //
  uart_gps.begin(GPSBAUD); //Sets baud rate of your GPS

  // Generate random run number //
  randomSeed(analogRead(A2)+analogRead(A5)); // use the wiggle on the analog pins to seed random
  // next we'll try analogRead(0) && analogRead(1) << 1 && analogRead(2) << 2 
  rand_num = abs(int(random(100,9999)));
  //RUN_NUMBER =  String(month) + String(day) + String(year) + String(rand_num);
  RUN_NUMBER = String(rand_num);

  // Get ID from prom
  String prom_str = "";
  for(int i=0; i<4; i++) {
    char curr_val = EEPROM.read(i);
    prom_str += curr_val;
  }
  DEVICE_ID = prom_str + "_" + VERSION + "_" + gps_interval; // Device ID

  /* Here we go through a couple of different start-up routines
   /  waiting for different conditions at each step */

  // Wait for data dump request //
  dataDump(); // see below for details

  // see if the card is present and can be initialized:
  while(!card_ok)
  {
    if (!SD.begin(chipSelect)) {
#ifdef DEBUG
      Serial.println("SD card failed, or not present");
#endif
      blinky(1,0,0,-1);
      //delay(1000);
      //blinky(0,0,0,-1);
      //delay(2000); // wait...
      //setup(); // ... then try again
      card_ok == false; // abort 
    } 
    else 
    {
#ifdef DEBUG
      Serial.println("SD card Ok");
#endif      
      card_ok = true;
      if(card_ok) {
        blinky(0,1,0,-1);
        dataFile = SD.open(file_name);
        if(!dataFile) {  // If there isn't a data file, open one and write a header
          dataFile = SD.open(file_name, FILE_WRITE); // open file for writing    
#ifdef DEBUG
          Serial.println("Writing file header");
#endif      
          String header = "ID,VERSION,INTERVAL,DATE,TIME,LAT,LON,ALTITUDE,BEARING,MPH,KMH,AIR,TEMP_C,TEMP_F,HUMIDITY,LUX";
          Serial.println(header);
          dataFile.close();
          delay(250);
        } 
        break;
      } 
    }
  }


  // Wait for satellite lock //
  while(!locked)
  { 
#ifdef DEBUG
    Serial.println("Waiting for lock...");
#endif

    if(fix_age > 5000.00)
    {
      blinky(0,1,0,125);
      //setup();
    } 
    else 
    {
      locked = true;
      break;
    }
    blinky(1,0,0,75); // 
    delay(500);
  }

  delay(1000);

  /* The very last thing we do is turn on the heater. Because if we haven't made it to here
   / somthing is wrond and we shouldn't turn on the heater to save on battery */
  //int heater_level = 255.0 * (100/heater_level_percent); // we PWM the heater element at a percent of power to save on battery (effects accuracy)
  //analogWrite(6, heater_level); 
  // Changed to stratight full power
  digitalWrite(6, HIGH);
}



//*** MAIN ***//
void loop()
{
  queryGps();
}


/* Query GPS module and return GPS data */
void queryGps()
{
  // Get GPS data //
  while(uart_gps.available())  // While there is data on the RX pin...
  {
    int c = uart_gps.read();    // load the data into a variable...
    if(gps.encode(c)) // if there is a new valid sentence...
    {
      getgps(gps);  // then grab the data.
      //delay(100);
      logData();   // log data
      delay(INTERVAL); // time between queries
    }
  }
  return;
}


// The getgps function will get and print the values we want.
void getgps(TinyGPS &gps)
{
  blinky(1,0,0,40);
  gps.f_get_position(&latitude, &longitude, &fix_age);
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
  altitude = gps.f_altitude();
  course = gps.f_course();
  mph = gps.f_speed_mph();  


  // Here you can print statistics on the sentences.
#ifdef DEBUG
  //Serial.println("Getting GPS...");

  unsigned long chars;
  unsigned short sentences, failed_checksum;
  gps.stats(&chars, &sentences, &failed_checksum);
  if(failed_checksum > 1) {
    Serial.println("GPS failed checksum");
    blinky(1,0,0,-1);
    //delay(2000);
    //Serial.println("restarting...");
    //setup();
  }
#endif
  return;
}


// Log Data //
void logData()
{
  // Make date and timestamp parts //
  dateStamp =  String(year) + "-" + String(month) + "-" + String(day); // changed in v0.6.1b for happier MySQL results. 
  // Pretty print minutes and seconds
  String min, sec = "";
  if(minute < 10) {
    min = "0"+String(minute);
  } 
  else {
    min = String(minute);
  }
  if(second < 10) {
    sec = "0"+String(second);
  } 
  else {
    sec = String(second);
  }
  timeStamp = String(hour) + ":" + min + ":" + sec;

#ifdef DEBUG
  Serial.println("\n" + timeStamp);
#endif

  //-- Get sonsor values --//

  AIR = analogRead(A1);
#ifdef DEBUG_SENSOR
  Serial.print("Air: "); 
  Serial.println(AIR);
#endif
  //temp.GetTemperature();
  //temp = dht.readTemperature(); // read temp F (can also be temp_c)
    TEMP_C = dht.getTemperatureC();
    TEMP_F = (TEMP_C * 1.8) + 32; // C * 9/5 + 32
    HUMIDITY = dht.getHumidity();
#ifdef DEBUG_SENSOR
  Serial.print("Temp: ");
  Serial.println(TEMP_C);
  Serial.print("Hum: ");
  Serial.println(HUMIDITY);
#endif

  LUX = analogRead(A0);
#ifdef DEBUG_SENSOR
  Serial.print("Light: "); 
  Serial.println(LUX);
#endif

  /*
  battery_lvl = analogRead(A2);
   #ifdef DEBUG_SENSOR
   Serial.print("Battery: "); 
   Serial.println(battery_lvl);
   #endif
   */

  // Add extra sensors to be read from here as needed (see --- Sensors --- below) //

  /* Write data to the SD card */
  dataFile = SD.open(file_name, FILE_WRITE);
  // if the file is available, write to it:
  if(dataFile) {
    blinky(0,1,0,35);
#ifdef DEBUG
    Serial.println("Writing data to card");
#endif

   dataFile.print(DEVICE_ID);
    dataFile.print(",");
    dataFile.print(VERSION);
    dataFile.print(",");
    dataFile.print(dateStamp); 
    dataFile.print(",");
    dataFile.print(timeStamp);
    dataFile.print(",");
    dataFile.print(dtostrf(latitude, 4, 6, new_lat));
    dataFile.print(",");
    dataFile.print(dtostrf(longitude, 4, 6, new_lon));
    dataFile.print(",");
    dataFile.print(altitude);
    dataFile.print(",");
    dataFile.print(int(course));
    dataFile.print(",");
    dataFile.print(int(mph));
    dataFile.print(",");
    dataFile.print(AIR); 
    dataFile.print(",");
    dataFile.print(TEMP_F); 
    dataFile.print(",");
    dataFile.print(HUMIDITY); 
    dataFile.print(",");
    dataFile.print(LUX); // note there is NO delimiter after the last value
    
    dataFile.println("");
    dataFile.close();

    delay(250); // we delay here to wait for the close file handler

  }  
  else {
#ifdef DEBUG
    Serial.println("Failed to write to card");
#endif
    dataFile.close();
  }

  return;
}


// Function to flash LEDs for visual status //
void blinky(int R, int G, int B, int time)
{
#ifdef BLINK
  digitalWrite(RED_LED, R);
  //analogWrite(GRN_LED, G * 255);
  digitalWrite(GRN_LED, G);
  // digitalWrite(BLU_LED, B);
  if(time > 0) { // trick for using blinky(n,n,n,-1) so LEDs stay on
    delay(time);
    digitalWrite(RED_LED, 0);
    analogWrite(GRN_LED, 0);
  }
  //return;
#endif
}

void breathing(int breaths){
  // Start breathing seq
  for(int i=0; i <= 16*breaths; i++)
  {
    analogWrite(9, breath[i%16]);
    delay(150);
  }
}


/// Get a data dump request via serial ///
void dataDump()
{
  int counter = 0;
  while(Serial.available() == false) // wait for message via serial
  {
    blinky(1,1,0, 500);
    counter += 1;
    if(counter >= dump_retries) // wait N times for dump request...
    {
      return; // ...else return to setuo()
    }
    delay(1000); // delay between blinks
  } 

  if(Serial.available() > 0) { // if something does come in on serial...
    // First check card status
    if (!SD.begin(chipSelect)) {
      blinky(1,0,0,-1); // card error
    } 
    else 
    {
      blinky(0,1,0,-1); 
      /// Read from SD card 
      File dumpFile = SD.open(file_name);
      // 
      char ser_char = Serial.read();
      if(ser_char == 100) { // look for 100, ASCII "d" (for dump)
        // see if the card is present and can be initialized: 
        if(dumpFile) {
          // read from the file until there's nothing else in it:
          int byte_count = 0;
          while (dumpFile.available()) {
            Serial.write(dumpFile.read());
            byte_count++; // count how many bytes transmitted
          }
          /* might want to send some sort of end marker here 
           /  could also send a byte count as an integrity check
           / (not presently used) */
          dumpFile.close();  // close the file
          blinky(0,0,0,-1);
          while(true){
          } // infinite while
        }
      }
      if(ser_char == 120) { //  look for 120, ASCII "x" (clear file) 
        SD.remove(file_name); // remove file on demand
        dumpFile.close();  // close the file
        blinky(1,0,0,-1);
        Serial.println(0);
        while(true){
        } // infinite while
      } 
      if(ser_char != 100 || ser_char != 120) // if the wrong character is sent, try again
      {
        dataDump(); 
        blinky(0,0,0,-1);
      }
    }
  }
}









