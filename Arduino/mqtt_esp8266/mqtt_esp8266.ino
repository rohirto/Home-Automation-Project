/***************************************************
Home Automation Project
Author: rohirto
Hardware: NodeMCU 12-E, DHT11, BMP 280, RCWL 0516, 2 Relay module
GPIO Pins used: RCWL: D5
                DHT11: D6
                BMP280: D1, D2 -> I2C Interface
                Relay 2 pins: 
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Your SSID"
#define WLAN_PASS       "Your Password"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "username"
#define AIO_KEY         "key"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'temperature' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish altitude = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/altitude");
Adafruit_MQTT_Publish occupancy = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/occupancy");

// Setup a feed called 'Auto Mode' for subscribing to changes.
Adafruit_MQTT_Subscribe auto_mode = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/auto-mode");
Adafruit_MQTT_Subscribe switch_control_data = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/switch-board-control");
Adafruit_MQTT_Subscribe fan_control_data = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/fan-control");

/********************Sensors definations**************************/
// Doppler Sensor RCWL 0516
/* Test conditions: Sensor in "H mode"*/
#define TEMPERATURE 1
#define HUMIDITY    2
#define OCCUPANCY   3
const byte RCWL_input_pin = D5;

//DHT 11 Sensor
#include "DHTesp.h"
DHTesp dht;

// for DHT11, 
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 3
const byte pinDHT11 = D6;
float temperature_data = 0;
float humidity_data = 0;

//BMP280 Pressure and Altitude
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
float pressure_data = 0 , altitude_data = 0;

//Relay Pins
#define RELAY_PIN_SWITCH  D3
#define RELAY_PIN_FAN     D7

/************************Timer definations********************************/
unsigned long startMillis;  //Some global vaiable anywhere in program
unsigned long currentMillis;
volatile byte temp_humd_timer = 30;  // In 10 secs multiple //1 min timer 
volatile byte temp_humd_timer_elapsed = false;
volatile byte occupancy_timer_elapsed = false;
volatile byte ten_sec_counter = 0;
volatile byte occupancy_timer = 3;
/****************************************************************************/

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Home Automation Project"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for auto mode feed.
  mqtt.subscribe(&auto_mode);
  mqtt.subscribe(&switch_control_data);
  mqtt.subscribe(&fan_control_data);

  //Sensors Setup
  Sensors_setup();

  //Relay Setup
  Relay_setup();

  //Timer start
  startMillis = millis();
}


void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &auto_mode) {
      Serial.print(F("Auto mode: "));
      Serial.println((char *)auto_mode.lastread);
    }
    if (subscription == &switch_control_data) {
      //Whatever Control data has come assign it to pin
      //Serial.print(F("\n Switch Control Data:"));
      //Serial.println((char *)switch_control_data.lastread);
      if(strcmp((char *)switch_control_data.lastread, "1") == 0)
      {
        //ON command
        digitalWrite(RELAY_PIN_SWITCH, LOW);
      }
      else
      {
        //Off command
        digitalWrite(RELAY_PIN_SWITCH, HIGH);
      }
    }    
   if (subscription == &fan_control_data) {
      //Whatever Control data has come assign it to pin
      if(strcmp((char *)fan_control_data.lastread, "1") == 0)
      {
        //ON command
        digitalWrite(RELAY_PIN_FAN, LOW);
        //Serial.println("Hit!");
      }
      else
      {
        //Off command
        digitalWrite(RELAY_PIN_FAN, HIGH);
      }
    } 
  }   

  //Update Timers  
  timer_function();

  //PIR auto mode related stuff
  if(occupancy_timer_elapsed == true)
  {
    Serial.print(F("\nSending occupancy val "));
    Serial.println((digitalRead(RCWL_input_pin)));
    
    if (! occupancy.publish(digitalRead(RCWL_input_pin))) 
    {
      Serial.println(F("Failed"));
    } 
    else
    {
      Serial.println(F("OK!"));
    }
    occupancy_timer_elapsed = false;
  }
    

  if(temp_humd_timer_elapsed == true)
  {
    DHT_11_loop();

    BMP_280_loop();

    //Post temperature data   
    Serial.print(F("\nSending temperature val "));
    Serial.print(temperature_data);
    Serial.print("...");
    if (! temperature.publish(temperature_data)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
    delay(500);
    Serial.print(F("\nSending humidity val "));
    Serial.print(humidity_data);
    Serial.print("...");
    if (! humidity.publish(humidity_data)) {
    Serial.println(F("Failed"));
    } else {
    Serial.println(F("OK!"));
    }
    //Pressure data
    Serial.print(F("\nSending Pressure val "));
    Serial.print(pressure_data);
    Serial.print("...");
    if (! pressure.publish(pressure_data)) {
    Serial.println(F("Failed"));
    } else {
    Serial.println(F("OK!"));
    }
    //Altitude data
    Serial.print(F("\nSending Altitude val "));
    Serial.print(altitude_data);
    Serial.print("...");
    if (! altitude.publish(altitude_data)) {
    Serial.println(F("Failed"));
    } else {
    Serial.println(F("OK!"));
    }

    temp_humd_timer_elapsed = false;
   }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}


void Sensors_setup()
{
  //Digital Pin 2 is input
  //DHT Setup
  dht.setup(pinDHT11, DHTesp::DHT11); // GPIO12

  //RCWL Motion Sensor/ Doppler Sensor
  pinMode(RCWL_input_pin, INPUT);

  //BMP280 Pressure and Altitude Setup
  bme.begin(0x76); //Start I2C at 0x76 address
}

void Relay_setup()
{
  //define relay pins and their state
  //Pin D7 and D8 as relay output
  //Pin D7: Control 1 (line 1) 
  //Pin 5: Control 2 (line 2)
  pinMode(RELAY_PIN_SWITCH, OUTPUT);
  digitalWrite(RELAY_PIN_SWITCH, HIGH);
  pinMode(RELAY_PIN_FAN, OUTPUT);
  digitalWrite(RELAY_PIN_FAN, HIGH);
}

void DHT_11_loop()
{
  humidity_data = dht.getHumidity();
  temperature_data = dht.getTemperature();
  
}

void BMP_280_loop()
{
  pressure_data = bme.readPressure()/100;
  altitude_data = bme.readAltitude(1013.25);
}


void timer_function()
{
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if( currentMillis - startMillis >= 10000)
  {
    startMillis = currentMillis;
    ten_sec_counter++;

     if((ten_sec_counter % occupancy_timer) == 0)
    {
      occupancy_timer_elapsed = true;
    }
    if((ten_sec_counter % temp_humd_timer) == 0)  //test whether the period has elapsed
    {
      temp_humd_timer_elapsed = true;
      ten_sec_counter = 0;  //IMPORTANT to save the start time of the current LED state.
    }   
  }
 
}
