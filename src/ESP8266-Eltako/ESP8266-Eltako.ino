
/**
   SCOPE
   Reads weather values from weather station "Eltako Multisensor MS" (http://www.eltako.com/fileadmin/downloads/en/_datasheets/Datasheet_MS.pdf)
   using RS485 bus, then sends them over WiFi to ThingSpeak channel.

   NOTES
   - For serial upload to NodeMCU: unplug RX pin, or else upload will fail with error "espcomm_upload_mem failed".

   CREATED: August 26, 2018
*/

// Uncommment following for debug output via hardware serial bus for debugging purposes
// Can only be used on Arduinos with two hardware serial bus (e.g. Arduino mega)
// Settings Arduino IDE:
// - Tools > Debug port: Disabled
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#else
#define DEBUG_PRINT(x)
#endif

#ifdef DEBUG
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINTLN(x)
#endif

#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include "PrivateEnvVariables.h"

#define dlytime 10
#define readInterval 1000      // read from serial every x milliseconds
#define readToTrxRatio 59      // number of read cycles to wait before transmitting data
#define trxRetryDelay 2000     // delay between trx attempts to ThingSpeak
#define tsHTTP_OK 200          // HTTP 200 return code from ThingSpeak

#define DHTPIN 4      // DHT Sensor connected to digital pin 2.
#define DHTTYPE DHT11 // Type of DHT sensor.

#define BUFFER_SIZE 64
#define MSG_SIZE 40

// Web server for debugging
/* #include <Ethernet.h>
  byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01
  };
  EthernetServer server(80);
*/

// Initialize the Wifi client library. Necessary for ThingSpeak to work.
WiFiClient client;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

char recv_str[BUFFER_SIZE];
long rssi;
int tsReturnCode;
int trxRetries;

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

int readCycles = 0;
int readErrorCount = 0;
float temperatureInternal = 0;
float umidityInternal = 0;
boolean prevRainValue = false;
float windspeedAlertLimit = 10;

unsigned long previousReadMillis = 0;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;

boolean check_checksum(char *string)
{
  // true: calculated checksum equals transferred checksum
  // false: calculated checksum != transferred checksum

  char buffer[5];
  boolean ret_val;
  checksum = 0;
  recv_sum = 0;
  for (int i = 0; i < MSG_SIZE - 5; i++) checksum += string[i]; // calculate the checksum
  strncpy (buffer, string + MSG_SIZE - 5, 4);
  buffer[4] = '\0';
  recv_sum = atoi(buffer); // convert transferred checksum to integer
  DEBUG_PRINTLN ("checksum: " + checksum);
  DEBUG_PRINTLN ("recv_sum:" + recv_sum);
  if (recv_sum == checksum) {
    ret_val = true;
    delay (dlytime);
  }
  else {
    ret_val = false;
    delay (dlytime);
  }
  return (ret_val);
}


void convert_rcvstr (char *string)
// split string into single values
{
  char buffer[6];
  if (check_checksum (string)) {
    strncpy (buffer, string + 1, 5);
    buffer[5] = '\0';
    temperature = atof (buffer);
    DEBUG_PRINTLN ("temperature: " + temperature);
    strncpy (buffer, string + 6, 2);
    buffer[2] = '\0';
    sun_south = atoi (buffer);
    DEBUG_PRINTLN ("sun_south: " + sun_south);
    strncpy (buffer, string + 8, 2);
    buffer[2] = '\0';
    sun_west = atoi (buffer);
    DEBUG_PRINTLN ("sun_west: " + sun_west);
    strncpy (buffer, string + 10, 2);
    buffer[2] = '\0';
    sun_east = atoi (buffer);
    DEBUG_PRINTLN ("sun_east: " + sun_east);
    string [12] == 'J' ? dawn = true : dawn = false;
    DEBUG_PRINTLN ("dawn: " + dawn);
    strncpy (buffer, string + 13, 3);
    buffer[3] = '\0';
    daylight = atoi (buffer);
    DEBUG_PRINTLN ("daylight: " + daylight);
    strncpy (buffer, string + 16, 4);
    buffer[4] = '\0';
    windspeed = atof (buffer);
    DEBUG_PRINTLN ("windspe: " + windspeed);
    string [20] == 'J' ? rain = true : rain = false;
    DEBUG_PRINTLN ("rain: " + rain);
  }
}


void read_serial()
{
  int index;
  DEBUG_PRINTLN ("Serial Buffer: " + Serial.available());
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
        readErrorCount++;
        break;
      }
    }
    delay (dlytime);
    digitalWrite(2, HIGH);  // Data transmission has ended --> turn off ESP-12 LED
    delay (dlytime);
    recv_str[index] = '\0';
    DEBUG_PRINTLN ("recv_str: " + recv_str);
    convert_rcvstr (recv_str);
  }
}


void updateDHT()
// Get the most recent readings for temperature and humidity.
{
  // To return Fahrenheit use dht.readTemperature(true)
  temperatureInternal = dht.readTemperature();
  umidityInternal = dht.readHumidity();
  DEBUG_PRINTLN ("temperatureInternal: " + temperatureInternal);
  DEBUG_PRINTLN ("umidityInternal: " + umidityInternal );
}


int connectWifi()
{
  digitalWrite(LED_BUILTIN, LOW);  // BUILTIN LED on to indicate WiFi connection in progress
  WiFi.begin(ssid, wifiPwd);
  while (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_PRINTLN ( "Connecting to WiFi..." );
    delay(2500);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}


// Web server for debugging
/* void listenForWebClients() {
  //listen for incoming web clients
  char buffer[90];

  EthernetClient client = server.available();
  if (client) {
    DEBUG_PRINTLN ("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.print("HTTP/1.1 200 OK\nContent-Type: text/html\nConnection: close\nRefresh: 5\n\n<!DOCTYPE HTML>");
          snprintf (buffer, 90, "<html>\nRawData.value %s<br>\n", recv_str);
          client.print(buffer);
          snprintf (buffer, 90, "CheckSum.value %d<br>\nRecvSum.value %d<br>Temperature.value ", checksum, recv_sum);
          client.print(buffer);
          client.print(temperature);
          snprintf (buffer, 90, "<br>Daylight.value %d<br>\nDawn.value %d<br>\nRain.value %d<br>\nWindspeed.value ", daylight, dawn, rain);
          client.print(buffer);
          client.print (windspeed);
          snprintf (buffer, 90, "<br>SunSouth.value %d<br>\nSunWest.value %d<br>\nSunEast.value %d<br>", sun_south, sun_west, sun_east);
          client.print(buffer);
          client.print("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
  }
*/


void setup()
{
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(2, OUTPUT);               // Initialize GPIO2 pin as an output
  digitalWrite(LED_BUILTIN, HIGH);  // Turn off LED_BUILTIN
  digitalWrite(2, HIGH);            // Turn off GPIO2
  DEBUG_PRINTLN ( "Starting" );

  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP & event)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn  BUILTIN LED off to indicate WiFi connection
    DEBUG_PRINTLN ("Connected to Wifi with IP: " + WiFi.localIP());
  });

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected & event)
  {
    // digitalWrite(LED_BUILTIN, LOW);  // BUILTIN LED on to indicate no WiFi connection
    DEBUG_PRINTLN ( "Disconnected from WiFi" );
    connectWifi();
  });

  connectWifi();

  ThingSpeak.begin(client);

}



void loop()
{

  unsigned long currentReadMillis = millis();

  if (currentReadMillis - previousReadMillis > readInterval) {
    previousReadMillis = currentReadMillis;

    // Read the weather values
    read_serial(); // read the serial bus

    // Get WiFi signal strength
    rssi = WiFi.RSSI();

    // Read internal temperature and humidity
    updateDHT();

    // Transmit values to ThingSpeak readToTrxRatio cycles
    // If it starts raining, or wind is above 10 m/s: transmit values after 1 read cycle
    if ( (readCycles >= readToTrxRatio) || ((rain == true) && (prevRainValue == false)) || ( windspeed > windspeedAlertLimit) ) {
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
        if ( tsReturnCode != tsHTTP_OK ) delay(trxRetryDelay);
      } while ( (tsReturnCode != tsHTTP_OK) || (trxRetries > 3) );

      ThingSpeak.setField(1, rssi);
      ThingSpeak.setField(2, temperatureInternal);
      ThingSpeak.setField(3, umidityInternal);
      ThingSpeak.setField(4, readErrorCount);

      trxRetries = 0;
      do {
        tsReturnCode = ThingSpeak.writeFields(tsInternalsChannelID, tsInternalsWriteAPIKey);
        trxRetries++;
        if ( tsReturnCode != tsHTTP_OK ) delay(trxRetryDelay);
      } while ( (tsReturnCode != tsHTTP_OK) || (trxRetries > 3) );

      readCycles = 0;
    }
    readCycles++;
    prevRainValue = rain;
  }
  // Web server for for debugging
  /* listenForWebClients(); */
}

