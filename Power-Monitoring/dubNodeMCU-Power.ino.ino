/*
 *  This sketch sends ads1115 current sensor data via HTTP POST request to thingspeak server.
 *  It needs the following libraries to work (besides the esp8266 standard libraries supplied with the IDE):
 *
 *  - https://github.com/adafruit/Adafruit_ADS1X15
 *
 *  designed to run directly on esp8266-01 module, to where it can be uploaded using this marvelous piece of software:
 *
 *  https://github.com/esp8266/Arduino
 *
 *  2015 Tisham Dhar
 *  licensed under GNU GPL
 */

/*
 *  Sketch Adapted by Award Digital - Warwick Ward 2016
 *  
 *  Additions
 *  - Thingspeak Library
 *  - CloudMQTT Integration for heartbeat monitoring
 *  - BUILTIN_LED Heartbeat for visial monitoring
 *  - Arduino BasicOTA sketch publishing
 */

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ThingSpeak.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

//WIFI credentials go here
const char* ssid     = "2coolforschool";
const char* password = "marie123";
const char* host = "dubNodeMCU-Power";

//PubSubClient Setup
const char* mqtt_server = "m10.cloudmqtt.com";
WiFiClient espClient;
PubSubClient client(espClient);

//THINKSPEAK SETUP
unsigned long tsChannelNumber = 83502;
const char * tsWriteAPIKey = "34QYYCPOIABHWW0R";

//HEARTBEAT CONFIG
int ledState = HIGH;
unsigned long previousMillisHeartbeat = 0;   
const long intervalHeartbeat = 2000;       

//SENSOR READ INTERVAL
unsigned long previousMillisSensor = 0;   
const long intervalSensor = 20000; 

//---------------------------------------------
//ADS 
//--------------------------------------------
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Maximum value of ADS
#define ADC_COUNTS 32768
#define PHASECAL 1.7
#define VCAL 0.6
#define ICAL 0.003

//-----------------------------------------
// MQTT PubSubClient
//-----------------------------------------
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
    client.publish("/device/dubnodemcu-power/out", "1");
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
      client.publish("/device/dubnodemcu-power/out", "1");
      // ... and resubscribe
      //client.subscribe("inTopic");
      client.subscribe("/device/dubnodemcu-power/in");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


//-----------------------------------------
// POWER READINGS AND CALCULATIONS
//-----------------------------------------
double filteredI;
double lastFilteredV,filteredV; //Filtered_ is the raw analog value minus the DC offset
int sampleV;                 //sample_ holds the raw analog read value
int sampleI; 

double offsetV;                          //Low-pass filter output
double offsetI;                          //Low-pass filter output

double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
double phaseShiftedV; //Holds the calibrated phase shifted voltage.
int startV; //Instantaneous voltage at start of sample window.
double sqV,sumV,sqI,sumI,instP,sumP; //sq = squared, sum = Sum, inst = instantaneous
boolean lastVCross, checkVCross; //Used to measure number of times threshold is crossed.

  double squareRoot(double fg)  
  {
    double n = fg / 2.0;
    double lstX = 0.0;
    while (n != lstX)
    {
      lstX = n;
      n = (n + fg / n) / 2.0;
    }
    return n;
  }

void calcVI(unsigned int crossings, unsigned int timeout)
{

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented  

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st=false;                                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(st==false)                                   //the while loop...
  {
     startV = ads.readADC_Differential_2_3();                    //using the voltage waveform
     if ((abs(startV) < (ADC_COUNTS*0.55)) && (abs(startV) > (ADC_COUNTS*0.45))) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
  }
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //------------------------------------------------------------------------------------------------------------------------- 
  start = millis(); 

  while ((crossCount < crossings) && ((millis()-start)<timeout)) 
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = ads.readADC_Differential_2_3();                 //Read in raw voltage signal
    sampleI = ads.readADC_Differential_0_1();                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;
   
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleV > startV) checkVCross = true; 
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;                  
                     
    if (lastVCross != checkVCross) crossCount++;
  }
 
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied. 
  float multiplier = 0.125F; /* ADS1115 @ +/- 4.096V gain (16-bit results) */
  double V_RATIO = VCAL * multiplier;
  Vrms = V_RATIO * squareRoot(sumV / numberOfSamples); 
  
  double I_RATIO = ICAL * multiplier;
  Irms = I_RATIO * squareRoot(sumI / numberOfSamples); 

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
  //--------------------------------------------------------------------------------------       
  }
  
  double calcIrms(unsigned int Number_of_Samples)
  {
    /* Be sure to update this value based on the IC and the gain settings! */
    float multiplier = 0.125F;    /* ADS1115 @ +/- 4.096V gain (16-bit results) */
    for (unsigned int n = 0; n < Number_of_Samples; n++)
    {
      sampleI = ads.readADC_Differential_0_1();
  
      // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, 
    //  then subtract this - signal is now centered on 0 counts.
      offsetI = (offsetI + (sampleI-offsetI)/1024);
      filteredI = sampleI - offsetI;
      //filteredI = sampleI * multiplier;
  
      // Root-mean-square method current
      // 1) square current values
      sqI = filteredI * filteredI;
      // 2) sum 
      sumI += sqI;
    }
    
    Irms = squareRoot(sumI / Number_of_Samples)*multiplier; 
  
    //Reset accumulators
    sumI = 0;
  //--------------------------------------------------------------------------------------       
 
  return Irms;
}

void setup() {

  //------------------------------------
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

  //-----------------------------------
  // OTA MANAGEMENT
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  ArduinoOTA.setHostname(host);
  // No authentication by default-
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

  //---------------------------------------------
  //HEARTBEAT LED
  pinMode(BUILTIN_LED, OUTPUT);     // Initializethe BUILTIN_LED pin as an output
  
  //--------------------------------------------
  //PubSubClient
  client.setServer(mqtt_server, 12881);
  client.setCallback(callback);
  
  //---------------------------------------------
  //THINGSPEAK
  ThingSpeak.begin(espClient);
  
  //---------------------------------------------
  //ADS
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  ads.begin();
  
}


void loop() {

  //-------------------------------------
  //OTA CHECK
  ArduinoOTA.handle();

  //-------------------------------------
  //PubSubClient
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  //-------------------------------------
  // HEARTBEAT 2 SECTION PULSE
  unsigned long currentMillisHeartbeat = millis();
  if (currentMillisHeartbeat - previousMillisHeartbeat >= intervalHeartbeat) {
    previousMillisHeartbeat = currentMillisHeartbeat;
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(BUILTIN_LED, ledState);
    Serial.println("Heartbeat msg: dubNodeMCU-Power");
  }
     
  //--------------------------------------
  //SENSOR READ SECTION EVERY SET PERIOD
  unsigned long currentMillisSensor = millis();
  if (currentMillisSensor - previousMillisSensor >= intervalSensor) {
    previousMillisSensor = currentMillisSensor;

    //-----------------------------------
    //POWER READING AND CALCULATIONS
    //Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(trans_volt); Serial.println("mV)");
    double current = calcIrms(2048);
    Serial.print("Just Current:");
    Serial.println(current);
    calcVI(20,2000); 
    Serial.print("Real Power:");
    Serial.println(realPower);
    Serial.print("Irms:");
    Serial.println(Irms);
    Serial.print("Vrms:");
    Serial.println(Vrms);

    //--------------------------------
    //THINGSPEAK UPLOAD DATA
    ThingSpeak.setField(1,String(realPower));
    ThingSpeak.setField(2,String(Irms));
    ThingSpeak.setField(3,String(Vrms));
    ThingSpeak.setField(4,String(powerFactor));
    ThingSpeak.writeFields(tsChannelNumber, tsWriteAPIKey);
  }
}
