
/**
   SCOPE
   Read weather values from weather station "Eltako Multisensor MS" (http://www.eltako.com/fileadmin/downloads/en/_datasheets/Datasheet_MS.pdf)
   using RS485 bus, then send them over WiFi to ThingSpeak channel.

   NOTES
   - For serial upload to NodeMCU: unplug RX pin, or else upload will fail with error "espcomm_upload_mem failed".

   CREATED: August 26, 2018

   ISSUES
   - [Solved] A but w/ Board Manager v2.4.1 would consume 56 Bytes of memory on every call of ThingSpeak.writeFields()
              (https://github.com/esp8266/Arduino/issues/4497). Used Serial.println("Free Heap: " + String( ESP.getFreeHeap() )
              to diagnose memory use.
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include "PrivateEnvVariables.h"

#define BUFFER_SIZE 64
#define MSG_SIZE 40
#define DHTPIN 4                      // DHT Sensor connected to digital pin 2
#define DHTTYPE DHT11                 // Type of DHT sensor
#define DELAY_MS 10                   // Delay in ms
#define READ_INTERVAL 1000            // Read from serial every x ms
#define READ_TO_TRX_RATIO 59          // Number of read cycles to wait before transmitting data
#define TRX_RETRY_DELAY 5000          // Delay between transmission attempts to ThingSpeak in ms
#define TS_HTTP_OK 200                // HTTP 200 return code from ThingSpeak
#define TS_MIN_UPDATE_FREQUENCY 20    // Number of read intervals to wait before sending new data to ThingSpeak (which would throw an error otherwise)
#define WINDSPEED_LIMIT 8.0           // Wind speed limit above which data is immediately sent to ThingSpeak
#define MEM_LOWER_LIMIT 10000         // Heap memory limit under which ESP is restarted


// Initialize the Wifi client library. Necessary for ThingSpeak to work.
WiFiClient client;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// Initialize web server and over-the-air HTTP update server
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

char recv_str[BUFFER_SIZE];

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
boolean prevRainValue = false;
//float WINDSPEED_LIMIT = 8;

unsigned short int readCycles = 0;
unsigned long previousReadMillis = 0;


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


void read_serial()
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
// Get the most recent readings for temperature and humidity.
{
  // To return Fahrenheit use dht.readTemperature(true)
  temperatureInternal = dht.readTemperature();
  umidityInternal = dht.readHumidity();
}


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

  // Over-the-air HTTP update service, initialization sequence
  MDNS.begin(host);
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
}


void loop()
{
  long rssi;
  int upTime;
  boolean strongWind;
  boolean prevStrongWind;
  unsigned short int freeMem;
  unsigned short int tsReturnCode;
  unsigned long currentReadMillis;
  unsigned int trxRetries;
  float avgTemp;
  float avgWind;
  float maxWind;
  float tempValues[READ_TO_TRX_RATIO];
  float windValues[READ_TO_TRX_RATIO];
  boolean rainValues[READ_TO_TRX_RATIO];

  // Client for over-the-air HTTP image update service
  httpServer.handleClient();

  currentReadMillis = millis();

  if (currentReadMillis - previousReadMillis >= READ_INTERVAL) {
    previousReadMillis = currentReadMillis;

    read_serial();
    // Store values in arrays
    tempValues[readCycles] = temperature;
    windValues[readCycles] = windspeed;
    rainValues[readCycles] = rain;

    // Transmit values to ThingSpeak only from dawn till dusk
    if ( daylight > 0 ) {

      // Read internal temperature and humidity
      updateDHT();

      // Get WiFi signal strength
      rssi = WiFi.RSSI();

      // Get free memory and restart ESP if below limit
      freeMem = ESP.getFreeHeap();
      if ( freeMem <= MEM_LOWER_LIMIT ) ESP.restart();

      // Calculate uptime
      upTime = round( currentReadMillis / (1000 * 60) );

      // Determine whether there's strong wind
      if (windspeed > WINDSPEED_LIMIT) strongWind = true;

      // Transmit values to ThingSpeak every READ_TO_TRX_RATIO cycles
      // If it starts raining or there is strong wind: transmit values as soon as possible, but at least TS_MIN_UPDATE_FREQUENCY cycles after last transmission
      if ( (readCycles >= READ_TO_TRX_RATIO) ||
           ( (rain == true) && (prevRainValue == false) && (readCycles >= TS_MIN_UPDATE_FREQUENCY) ) ||
           ( (strongWind == true) && (prevStrongWind == false) && (readCycles >= TS_MIN_UPDATE_FREQUENCY)) ) {

        ThingSpeak.setField(1, temperature);
        ThingSpeak.setField(2, daylight);
        ThingSpeak.setField(3, dawn);
        ThingSpeak.setField(4, rain);
        ThingSpeak.setField(5, windspeed);
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

        readCycles = 0;

        // Flush weather values
        for (int i = 0; i < READ_TO_TRX_RATIO; i++) {
          tempValues[i] = 0;
          windValues[i] = 0;
          rainValues[i] = false;
        }
      }
    }
    readCycles++;
    prevRainValue = rain;
    prevStrongWind = strongWind;
  }
  // Web server for for debugging
  /* listenForWebClients(); */
}

