#include <Arduino.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <NewPing.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

// WIFI SETTINGS
const char* ssid = "2coolforschool";
const char* password = "marie123";
const char* host = "dubNodeMCU-WaterTank";

//PubSubClient Setup
const char* mqtt_server = "m10.cloudmqtt.com";
WiFiClient espClient;
PubSubClient client(espClient);


//THINKSPEAK SETUP
unsigned long tsChannelNumber = 70809;
const char * tsWriteAPIKey = "9WX95THO4VA5U808";

//HEARTBEAT CONFIG
int ledState = HIGH;
unsigned long previousMillisHeartbeat = 0;   
const long intervalHeartbeat = 2000;       

//SENSOR READ INTERVAL
unsigned long previousMillisSensor = 0;   
const long intervalSensor = 30000; 

//PubSubClient
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
    client.publish("/device/dubesp-watertank/out", "1");
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

//PubSubClient
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client","water","water")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("/device/dubesp-watertank/out", "1");
      // ... and resubscribe
      //client.subscribe("inTopic");
      client.subscribe("/device/dubesp-watertank/in");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//DHT22
#define DHTPIN 0
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//RESULTS SMOOTHING
const int numReadings = 10;
int warmUpLogging = 0;
//Temp
int readingsTemp[numReadings];      // the readings from the analog input
int readIndexTemp = 0;              // the index of the current reading
int totalTemp = 0;                  // the running total
int averageTemp = 0;                // the average
//Humid
int readingsHumid[numReadings];      // the readings from the analog input
int readIndexHumid = 0;              // the index of the current reading
int totalHumid = 0;                  // the running total
int averageHumid = 0;                // the average
//Index
int readingsIndex[numReadings];      // the readings from the analog input
int readIndexIndex = 0;              // the index of the current reading
int totalIndex = 0;                  // the running total
int averageIndex = 0;                // the average
//Water
int readingsWater[numReadings];      // the readings from the analog input
int readIndexWater = 0;              // the index of the current reading
int totalWater = 0;                  // the running total
int averageWater = 0;                // the average

//BMP180
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

//HC-SR04 PINS & VARIABLE
#define TRIGGER_PIN 14
#define ECHO_PIN 12
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int waterLevel = 0;

void setup() {
  //WIFI SETUP
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // OTA MANAGEMENT
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  ArduinoOTA.setHostname(host);

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

  //HEARTBEAT LED
  pinMode(BUILTIN_LED, OUTPUT);     // Initializethe BUILTIN_LED pin as an output

  //PubSubClient
  client.setServer(mqtt_server, 12881);
  client.setCallback(callback);

  //DHT22
   dht.begin();

   //BMP180
  if(!bmp.begin())
  {
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  displaySensorDetails();

  //THINGSPEAK
   ThingSpeak.begin(espClient);

  //SMOOTHING - INITIALISE READING TO 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsTemp[thisReading] = 0;
    readingsHumid[thisReading] = 0;
    readingsIndex[thisReading] = 0;
    readingsWater[thisReading] = 0;
  }
}


void loop() {
  //OTA CHECK
  ArduinoOTA.handle();

  //PubSubClient
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // HEARTBEAT 2 SECTION PULSE
    unsigned long currentMillisHeartbeat = millis();
     if (currentMillisHeartbeat - previousMillisHeartbeat >= intervalHeartbeat) {
      previousMillisHeartbeat = currentMillisHeartbeat;
  
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      // set the LED with the ledState of the variable:
      digitalWrite(BUILTIN_LED, ledState);
      Serial.println("Heartbeat msg: dubESP-watertank");
     }
     
    unsigned long currentMillisSensor = millis();
   if (currentMillisSensor - previousMillisSensor >= intervalSensor) {
    previousMillisSensor = currentMillisSensor;
  
  //DHT22 SENSOR READING
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
  
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C ");
    //Serial.print(f);
    //Serial.print(" *F\t");
    Serial.print("Heat index: ");
    Serial.print(hic);
    Serial.println(" *C ");
    //Serial.print(hif);
    //Serial.println(" *F");

    //SMOOTHING-RESULT-TEMERATURE
    
    totalTemp = totalTemp - readingsTemp[readIndexTemp]; // subtract the last reading
    readingsTemp[readIndexTemp] = (int)t; // read from the sensor:
    totalTemp = totalTemp + readingsTemp[readIndexTemp];// add the reading to the total:
    readIndexTemp = readIndexTemp + 1; // advance to the next position in the array:
        
    if (readIndexTemp >= numReadings) { // if we're at the end of the array...
        readIndexTemp = 0; // ...wrap around to the beginning:
    }

    //SMOOTHING-RESULT-HUMIDITY
    
    totalHumid = totalHumid - readingsHumid[readIndexHumid]; // subtract the last reading
    readingsHumid[readIndexHumid] = (int)h; // read from the sensor:
    totalHumid = totalHumid + readingsHumid[readIndexHumid];// add the reading to the total:
    readIndexHumid = readIndexHumid + 1; // advance to the next position in the array:
        
    if (readIndexHumid >= numReadings) { // if we're at the end of the array...
        readIndexHumid = 0; // ...wrap around to the beginning:
    }

    //SMOOTHING-RESULT-HEATINDEX
    
    totalIndex = totalIndex - readingsIndex[readIndexIndex]; // subtract the last reading
    readingsIndex[readIndexIndex] = (int)hic; // read from the sensor:
    totalIndex = totalIndex + readingsIndex[readIndexIndex];// add the reading to the total:
    readIndexIndex = readIndexIndex + 1; // advance to the next position in the array:
        
    if (readIndexIndex >= numReadings) { // if we're at the end of the array...
        readIndexIndex = 0; // ...wrap around to the beginning:
    }

    averageTemp = totalTemp / numReadings;       // calculate the average:
    averageHumid = totalHumid / numReadings;       // calculate the average:
    averageIndex = totalIndex / numReadings;       // calculate the average:
    //Serial.print("Average Temp: ");
    //Serial.println(averageTemp);       // send it to the computer as ASCII digits
    //Serial.print("Average Humidity: ");
    //Serial.println(averageHumid);       // send it to the computer as ASCII digits
    //Serial.print("Average Heat Index: ");
    //Serial.println(averageIndex);       // send it to the computer as ASCII digits


    //BMP180 SENSOR READING
    sensors_event_t event;
    bmp.getEvent(&event);
   
    if (event.pressure)
    {
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");

    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    float seaLevelPressure = 1009;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure)); 
    Serial.println(" m");
    Serial.println("");
      }
    else
    {
    Serial.println("Sensor error");
    }

    //HC-SR04-READING
    unsigned int uS = sonar.ping_cm();
    Serial.print(uS);
    Serial.println("cm");
    
    //HC-SR04-SMOOTHING-RESULT
    totalWater = totalWater - readingsWater[readIndexWater]; // subtract the last reading
    readingsWater[readIndexWater] = uS; // read from the sensor:
    totalWater = totalWater + readingsWater[readIndexWater];// add the reading to the total:
    readIndexWater = readIndexWater + 1; // advance to the next position in the array:
        
    if (readIndexWater >= numReadings) { // if we're at the end of the array...
        readIndexWater = 0; // ...wrap around to the beginning:
    }

    averageWater = totalWater / numReadings;       // calculate the average:
    //Serial.print("Water Distance Average: ");
    //Serial.println(averageWater);       // send it to the computer as ASCII digits

    //WATER-TANK-LEVEL
    waterLevel = 110 - averageWater; // Change from distance from sensor to water level - Tank height 100cm
    Serial.print("Tank Level: ");
    Serial.print(waterLevel);       // send it to the computer as ASCII digits
    Serial.println(" %\t");
    

    //THINGSPEAK WRTIE VALUES AFTER 10 READS
        if (warmUpLogging++ < 11){
      Serial.println("Thingspeak Values Uploading... still warming up...");
    } else {
      ThingSpeak.setField(1,averageTemp);
      ThingSpeak.setField(2,averageHumid);
      ThingSpeak.setField(3,averageIndex);
      ThingSpeak.setField(4,(int)event.pressure);
      ThingSpeak.setField(5,waterLevel);
      ThingSpeak.writeFields(tsChannelNumber, tsWriteAPIKey);
    }
    
  }
}

