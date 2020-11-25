# Home-Automation-Project
Home Automation Project of my Room based on NodeMCU , adafruit IO and MQTT

Hard ware: 
Microcontroller: NODEMCU 12-E
Sensors: DHT11 Temperature and Humidity Sensor, BMP280 Pressure and Altitude Sensor, RCWL 0516 Doppler Sensor (Motion Detection)
Outputs/Actuators: 2 Relay Module

Hardware Setup Explanation:
1. Power Source: Using a 5V 1A Travel Adaptor (Micro USB) to Power up the Project. The cable is split in between and two 5V outputs are 
                 are drawn (1 to Node MCU, other to Relay Module, this is done for Safety Purpose). On board voltage regulator shifts down
                 the voltage from 5V to 3.3V required for NODEMCU (ESP8266). Relay Part is kept at 5V.
2. Microcontroller System:    SENSOR/OUTPUT     PIN
                            1. DHT11            D6
                            2. BMP280           D1, D2 (SCL, SDA) I2C interface
                            3. RCWL 0516        D5
                            4. Relay Pins       D3, D7
                            
Software: Used Arduino IDE, 
  1. Initial Setup: Had to install NodeMCU board first (Boards Manager), Relevant Libraries (Adafruit Unified Library, Adafruit BME280                         Library, DHT Sensor Library for ESPx, Adafruit MQTT Library, ESP8266 WiFi Library)
  2. Cloud setup: Used Adafruit IO instance (Free). Sign up for it and Proceed, its quite intuitional. Create Feeds where data will go and                   come from. Used around 8 Feeds in Project
  
