
/**
   SCOPE
   Read weather values from weather station "Eltako Multisensor MS" (http://www.eltako.com/fileadmin/downloads/en/_datasheets/Datasheet_MS.pdf)
   using RS485 bus, then send them over WiFi to ThingSpeak channel.

   NOTES
   - For serial upload to NodeMCU: unplug RX pin, or else upload will fail with error "espcomm_upload_mem failed".

   CREATED: August 26, 2018

   ISSUES
   - [Solved] A bug w/ Board Manager v2.4.1 would consume 56 Bytes of memory on every call of ThingSpeak.writeFields()
              (https://github.com/esp8266/Arduino/issues/4497). Used Serial.println("Free Heap: " + String( ESP.getFreeHeap() )
              to diagnose memory use.
*/

#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include "PrivateEnvVariables.h"
// Libraries for web server and over-the-air HTTP update server.
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#define BUFFER_SIZE 64
#define MSG_SIZE 40
#define DHTPIN 4                      // DHT Sensor connected to digital pin 2
#define DHTTYPE DHT11                 // Type of DHT sensor
#define DELAY_MS 10                   // Delay in ms
#define READ_INTERVAL 1000            // Read from serial every x ms
#define READ_TO_TRX_RATIO 60          // Number of read cycles betweeen transmitting data to ThingSpeak
#define WEATHER_BUFFER 59             // Size of array to store weather values
#define TRX_RETRY_DELAY 5000          // Delay between transmission attempts to ThingSpeak in ms
#define TS_HTTP_OK 200                // HTTP 200 return code from ThingSpeak
#define MEM_LOWER_LIMIT 10000         // Heap memory limit under which ESP is restarted
#define WIND_ARRAY_FILLER -1.0        // Value to fill wind array with at initialization
#define MAX_REALISTIC_WINDSPEED 50   // Maximum realistic windspeed in m/s beyond which we assume reading is buggy


// Initialize the Wifi client library. Necessary for ThingSpeak to work.
WiFiClient client;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// Initialize web server and over-the-air HTTP update server
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

char recv_str[BUFFER_SIZE];
float windValues[WEATHER_BUFFER];
boolean rainValues[WEATHER_BUFFER];

float temperature = 0;
float windspeed = 0;
int sun_south = 0;
int sun_west = 0;
int sun_east = 0;
int daylight = 0;
boolean dawn = false;
boolean rain = false;

int recv_sum = 0;
int checksum = 0;

float temperatureInternal = 0;
float umidityInternal = 0;

unsigned short int readCycles = 0;
unsigned short int weatherArrayIndex = 0;
unsigned long previousReadMillis = 0;

// For debugging
// char webDebugOutput[10000];

// WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;

boolean check_checksum(char *string)
// Verify if calculated checksum equals transferred checksum (return true) or not (return false)
{
  char buffer[5];
  boolean ret_val;
  checksum = 0;
  recv_sum = 0;
  for (int i = 0; i < MSG_SIZE - 5; i++) checksum += string[i]; // calculate the checksum
  strncpy (buffer, string + MSG_SIZE - 5, 4);
  buffer[4] = '\0';
  recv_sum = atoi(buffer); // convert transferred checksum to integer

  if (recv_sum == checksum) {
    ret_val = true;
    delay (DELAY_MS);
  }
  else {
    ret_val = false;
    delay (DELAY_MS);
  }
  return (ret_val);
}


void convert_rcvstr (char *string)
// Split string into single values
{
  char buffer[6];
  if (check_checksum (string)) {
    strncpy (buffer, string + 1, 5);
    buffer[5] = '\0';
    temperature = atof (buffer);
    //Serial.println("temperature: " + String(temperature));
    strncpy (buffer, string + 6, 2);
    buffer[2] = '\0';
    sun_south = atoi (buffer);
    //Serial.println("sun_south: " + sun_south);
    strncpy (buffer, string + 8, 2);
    buffer[2] = '\0';
    sun_west = atoi (buffer);
    //Serial.println("sun_west: " + sun_west);
    strncpy (buffer, string + 10, 2);
    buffer[2] = '\0';
    sun_east = atoi (buffer);
    //Serial.println("sun_east: " + sun_east);
    string [12] == 'J' ? dawn = true : dawn = false;
    //Serial.println("dawn: " + dawn);
    strncpy (buffer, string + 13, 3);
    buffer[3] = '\0';
    daylight = atoi (buffer);
    //Serial.println("daylight: " + daylight);
    strncpy (buffer, string + 16, 4);
    buffer[4] = '\0';
    windspeed = atof (buffer);
    //Serial.println("windspe: " + String(windspeed));
    string [20] == 'J' ? rain = true : rain = false;
    //Serial.println("rain: " + rain);
  }
}


void readSerial()
// Read the weather values from the serial bus
{
  int index;
  //Serial.println("Serial Buffer: " + Serial.available());
  if (Serial.available() > 0) {
    index = 0;
    digitalWrite(2, LOW);   // ESP-12 LED on to indicate we're reading data on serial bus

    while (Serial.available() > 0) {
      char inByte = Serial.read();
      if (index < BUFFER_SIZE - 1) {
        recv_str[index] = inByte;
        index++;
      }
      else {
        //readErrorCount++;
        break;
      }
    }
    delay (DELAY_MS);
    digitalWrite(2, HIGH);  // Data transmission has ended --> turn off ESP-12 LED
    delay (DELAY_MS);
    recv_str[index] = '\0';
    //Serial.println("recv_str: " + String(recv_str));
    convert_rcvstr (recv_str);
  }
}


void updateDHT()
// Get the most recent readings for internal temperature and humidity.
{
  // To return Fahrenheit use dht.readTemperature(true)
  temperatureInternal = dht.readTemperature();
  umidityInternal = dht.readHumidity();
}


float maxWind()
// Return the maximum value in wind array
{
  float maxValue = windValues[0];

  for (size_t i = 1; i < WEATHER_BUFFER; ++i) {
    if ( windValues[i] > maxValue ) {
      maxValue = windValues[i];
    }
  }
  return maxValue;
}


float avgWind()
// Return the average value in wind array, excluding 'WIND_ARRAY_FILLER' from calculation
{
  float sumValue = 0;
  int sumElems = 0;

  for (size_t i = 0; i < WEATHER_BUFFER; ++i) {
    if ( windValues[i] != WIND_ARRAY_FILLER ) {
      sumValue += windValues[i];
      sumElems++;
    }
  }

  if ( sumElems > 0 ) {
    return ( sumValue / sumElems );
  } else {
    return 0;
  }
}


boolean itsRaining()
// Return true if at least one value in rain array is true, false otherwise
{
  for (size_t i = 0; i < WEATHER_BUFFER; ++i) {
    if ( rainValues[i] == true ) {
      return true;
    }
  }
  return false;
}

/*
  void webDebug()
  // Output string used for debugging purposes on call to http server
  {
  httpServer.send(200, "text/plain", webDebugOutput);
  }
*/

int connectWifi()
// Connect to Wifi
{
  digitalWrite(LED_BUILTIN, LOW);  // BUILTIN LED on to indicate WiFi connection in progress
  WiFi.begin(ssid, wifiPwd);
  while (WiFi.status() != WL_CONNECTED)
  {
    //Serial.println( "Connecting to WiFi..." );
    delay(2500);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}


void setup()
{
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(2, OUTPUT);               // Initialize GPIO2 pin as an output
  digitalWrite(LED_BUILTIN, HIGH);  // Turn off LED_BUILTIN
  digitalWrite(2, HIGH);            // Turn off GPIO2

  //Serial.println("Starting");

  /*
    gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP & event)
    {
      digitalWrite(LED_BUILTIN, HIGH); // Turn  BUILTIN LED off to indicate WiFi connection
      Serial.println("Connected to Wifi with IP: " + WiFi.localIP());
    });

    disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected & event)
    {
      // digitalWrite(LED_BUILTIN, LOW);  // BUILTIN LED on to indicate no WiFi connection
      Serial.println( "Disconnected from WiFi" );
      connectWifi();
    });
  */

  connectWifi();

  ThingSpeak.begin(client);

  // For debugging, return wind values through http server
  // httpServer.on("/debug", webDebug);

  // Over-the-air HTTP update service, initialization sequence
  MDNS.begin(host);
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  //Serial.println("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);

  // Intialite weather arrays
  for (size_t i = 0; i < WEATHER_BUFFER; ++i) {
    windValues[i] = WIND_ARRAY_FILLER;
    rainValues[i] = false;
  }
}


void loop()
{
  long rssi;
  int upTime;
  unsigned short int freeMem;
  unsigned short int tsReturnCode;
  unsigned long currentReadMillis;
  unsigned int trxRetries;

  // Client for both web server and over-the-air HTTP image update service
  httpServer.handleClient();

  currentReadMillis = millis();

  if (currentReadMillis - previousReadMillis >= READ_INTERVAL) {
    previousReadMillis = currentReadMillis;

    // Read serial bus
    readSerial();
    readCycles++;

    // Store wind, rain values in arrays
    if ( weatherArrayIndex > WEATHER_BUFFER ) {
      weatherArrayIndex = 0;
    }
    rainValues[weatherArrayIndex] = rain;
    // Sometimes windspeed reading is buggy and returns approx. 2000; if so, enter 0 in array instead of actual reading
    if ( (windspeed >= 0) && (windspeed < MAX_REALISTIC_WINDSPEED) ) {
      windValues[weatherArrayIndex] = windspeed;
    }
    else {
      windValues[weatherArrayIndex] = 0;
    }
    weatherArrayIndex++;

    // For debugging
    //sprintf(webDebugOutput + strlen(webDebugOutput), "After readSerial: %u %u %f \n", currentReadMillis, readCycles, windValues[readCycles]);

    // Transmit values to ThingSpeak every READ_TO_TRX_RATIO cycles and only from dawn till dusk
    if ( ( daylight > 0 ) && ( readCycles >= READ_TO_TRX_RATIO ) ) {

      // Read internal temperature and humidity
      updateDHT();

      // Get WiFi signal strength
      rssi = WiFi.RSSI();

      // Get free memory and restart ESP if below limit
      freeMem = ESP.getFreeHeap();
      if ( freeMem <= MEM_LOWER_LIMIT ) ESP.restart();

      // Calculate uptime
      upTime = round( currentReadMillis / (1000 * 60) );

      ThingSpeak.setField(1, temperature);
      ThingSpeak.setField(2, daylight);
      ThingSpeak.setField(3, itsRaining() );
      ThingSpeak.setField(4, maxWind() );
      ThingSpeak.setField(5, avgWind() );
      ThingSpeak.setField(6, sun_south);
      ThingSpeak.setField(7, sun_west);
      ThingSpeak.setField(8, sun_east);

      trxRetries = 0;
      do {
        tsReturnCode = ThingSpeak.writeFields(tsDataChannelID, tsDataWriteAPIKey);
        trxRetries++;
        if ( tsReturnCode != TS_HTTP_OK ) delay(TRX_RETRY_DELAY);
      } while ( not( (tsReturnCode == TS_HTTP_OK) || (trxRetries > 3)) ); // stays in loop until either HTTP response ok or more than x retries

      ThingSpeak.setField(1, rssi);
      ThingSpeak.setField(2, temperatureInternal);
      ThingSpeak.setField(3, umidityInternal);
      ThingSpeak.setField(4, upTime);
      ThingSpeak.setField(5, freeMem);

      trxRetries = 0;
      do {
        tsReturnCode = ThingSpeak.writeFields(tsInternalsChannelID, tsInternalsWriteAPIKey);
        trxRetries++;
        if ( tsReturnCode != TS_HTTP_OK ) delay(TRX_RETRY_DELAY);
      } while ( not( (tsReturnCode == TS_HTTP_OK) || (trxRetries > 3)) );

      // Flush weather arrays
      /*
        for (int i = 0; i < READ_TO_TRX_RATIO; ++i) {
        windValues[i] = WIND_ARRAY_FILLER;
        rainValues[i] = false;

        // For debugging
        //sprintf(webDebugOutput + strlen(webDebugOutput), "When flushing: %u %u %f \n", currentReadMillis, i, windValues[i]);

        }
      */
      //  readCycles = 0;
    }
    // Reset readCycles also while not transmitting to ThingSpeak to avoid it going beyond limit
    if ( readCycles >= READ_TO_TRX_RATIO ) {
      readCycles = 0;
    }
  }
}


