#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
// MQTT Client
#include <PubSubClient.h>
#include <PIDController.h>

#include <Preferences.h>

Preferences preferences;

#include "RollerControl.h"


struct Config {
  char ssid[64];
  char pwd[32];
  char host[32];
  long len;
  long len2;
  int maxpwm;
  int minpwm;
  int roomNum;
};

struct MotorStatus
{
  int runStatus;
  int32_t lastPosition;
  bool pendingChange=false;
};


 #define LED_BUILTIN 2

const char *filename = "/wificonf.json";  // <- SD library uses 8.3 filenames
const char *motor0stat = "/motor0.json";
const char *motor1stat = "/motor1.json";

IPAddress mqttServer=IPAddress(192,168,1,5);
const char* topicListen = "digitalcomm/101/#" ;

// outbound Mqtt messahe
#define MSG_BUFFER_SIZE  (200)
char msg[MSG_BUFFER_SIZE];

#define MSG_TOPIC_SIZE  200
char topic[MSG_BUFFER_SIZE];



Config config;                         // <- global configuration object

MotorStatus  motorStatus1;
MotorStatus  motorStatus2;

WiFiClient espClient;
PubSubClient client(espClient);


#ifndef STASSID
#define STASSID "xxxxx"
#define STAPSK  "yyyyy"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

// rotary encoder library
#include <ErriezRotaryFullStep.h>

int motTimerChannelA=3;
int motTimerChannelB=4;
int motTimerChannel2A=1;
int motTimerChannel2B=2;


//  config PCB MH-Mini for motor 2
/*
#define MOT2_PINA 17
#define MOT2_PINB 16
#define MOT2_PWM 3
// int motTimerChannel=0;
//PCB 
#define ENC2_PINA 18
#define ENC2_PINB 26


#define END_STOP2 13
#define PUSH_BUTTON2 33





// config PCB MH-Mini for motor 1

#define MOT_PINA 27
#define MOT_PINB 12
#define MOT_PWM 2

// int motTimerChannelA=3;
// int motTimerChannelB=4;

//PCB 
#define ENC_PINA 5
#define ENC_PINB 25



#define END_STOP 4
#define PUSH_BUTTON 32
*/

// config PCB Wroom module for motor 2

#define MOT2_PINA 27
#define MOT2_PINB 12
#define MOT2_PWM 3
// int motTimerChannel=0;
//PCB 
#define ENC2_PINA 26
#define ENC2_PINB 25


#define END_STOP2 13
#define PUSH_BUTTON2 33





//  config PCB for motor 1

#define MOT_PINA 16
#define MOT_PINB 17
#define MOT_PWM 2

// int motTimerChannelA=3;
// int motTimerChannelB=4;

//PCB 
#define ENC_PINA 5
#define ENC_PINB 18



#define END_STOP 4
#define PUSH_BUTTON 32


#define LIGHT_PIN 2



// motor related variables
enum motorState_t { IDDLE, MOPENING, MCLOSING,  HALL_STOP };
motorState_t motorStatus=IDDLE;



// button variables 
enum buttonCommand_t { GO_UP, STOP, GO_DOWN};
buttonCommand_t  buttonCommand=GO_DOWN;

RollerControl  rollerControl;
RollerControl  rollerControl1;

// #define PROC_PUB_PERIOD 250
// rotary encoder motor 1
RotaryFullStep rotary(ENC_PINA, ENC_PINB);
// rotary encoder motor 2
RotaryFullStep rotary2(ENC2_PINA, ENC2_PINB);
// rotary position for motor/encoder 1
volatile int32_t count = 0;
// rotary position for motor/encoder 2
volatile int32_t count2 = 0;


// Forward declaration
void rotaryInterrupt();
void rotaryInterrupt2();
// encoder definition 

uint32_t lastUpTime=0;
int secsCount=0;
bool fwd=true;
bool initState=true;

#include <OneButton.h>
// init one button button activelow + enable pullup
OneButton debouncer = OneButton(PUSH_BUTTON, true, true); 
OneButton debouncer2 = OneButton(PUSH_BUTTON2, true, true); 

 unsigned int rstCount=0;

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
ICACHE_RAM_ATTR
#endif
void rotaryInterrupt()
{
    int rotaryState;
   
    // Read rotary state (Counter clockwise) -2, -1, 0, 1, 2 (Clockwise)
    rotaryState = rotary.read();
    //  Serial.print(rotaryState);
    // Count up or down by using rotary speed
    if (rotaryState == 0) {
        // No change
        return;
    } else if (abs(rotaryState) >= 2) {
        count += rotaryState * 2;
    } else {
        count += rotaryState;
    }

    // Limit count to a minimum and maximum value
   /* if (count > 10000) {
        count = 10000;
    }
    if (count < 0) {
        count = 0;
    }
   */
}


#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
ICACHE_RAM_ATTR
#endif
void rotaryInterrupt2()
{
    int rotaryState;
   
    // Read rotary state (Counter clockwise) -2, -1, 0, 1, 2 (Clockwise)
    rotaryState = rotary2.read();
    // Serial.print(rotaryState);
    // Count up or down by using rotary speed
    if (rotaryState == 0) {
        // No change
        return;
    } else if (abs(rotaryState) >= 2) {
        count2 += rotaryState * 2;
    } else {
        count2 += rotaryState;
    }

    // Limit count to a minimum and maximum value
   /* if (count > 10000) {
        count = 10000;
    }
    if (count < 0) {
        count = 0;
    }
   */
}


void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

   //  little fs start
   if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");

    //Print the error on display
    delay(1000);
    return;
  }

  preferences.begin("roller32", false); 

  rstCount=preferences.getUInt("rstCnt",0);
  Serial.print("Soft restart count ");
  Serial.println(rstCount);
  rstCount++;
  preferences.putUInt("rstCnt", rstCount);
  preferences.end();

  
  
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);
    
  

  // now configure motor1 status
  // initial setup pins for motor 1
  pinMode(MOT_PINA,OUTPUT);
  pinMode(MOT_PINB,OUTPUT);
  // pinMode(MOT_PWM,OUTPUT);
  pinMode(ENC_PINA,INPUT_PULLUP);
  pinMode(ENC_PINB,INPUT_PULLUP);
  // initial setup pins for motor 2
  pinMode(MOT2_PINA,OUTPUT);
  pinMode(MOT2_PINB,OUTPUT);
 // pinMode(MOT2_PWM,OUTPUT);
  pinMode(ENC2_PINA,INPUT_PULLUP);
  pinMode(ENC2_PINB,INPUT_PULLUP);

  pinMode(LED_BUILTIN,OUTPUT);
  // button related to motor 1
  pinMode(END_STOP,INPUT_PULLUP);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);  
  // button related to motor 2
  pinMode(END_STOP2,INPUT_PULLUP);
  pinMode(PUSH_BUTTON2, INPUT_PULLUP);  
  digitalWrite(MOT_PWM,LOW);
  // PWM Setup needed on ESP32 - motor1
  ledcSetup(motTimerChannelA, 5000, 8);
  ledcSetup(motTimerChannelB, 5000, 8);
  // PWM Setup needed on ESP32 - motor2
  ledcSetup(motTimerChannel2A, 5000, 8);
  ledcSetup(motTimerChannel2B, 5000, 8);

   ledcAttachPin(MOT_PINA, motTimerChannelA);
   ledcAttachPin(MOT_PINB, motTimerChannelB);
   ledcAttachPin(MOT2_PINA, motTimerChannel2A);
   ledcAttachPin(MOT2_PINB, motTimerChannel2B);

  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);
  
  MotorStatus tempStatus;
  
  motorStatus1.runStatus=IDDLE;
  motorStatus1.pendingChange=false;
  if(rstCount<2)
  {
      // start after power resume, loading position from file
      Serial.println("Motor 1 status from file");
      loadMotorStatus(motor0stat, motorStatus1);
      count=motorStatus1.lastPosition;
      // save to prefs to avoid crashes impact for 
      saveMotor0Status(motorStatus1);
  }
  else
  {
    if(digitalRead(END_STOP)==LOW)
    {
      motorStatus1.lastPosition=0;
    }
    else 
    {
      motorStatus1.lastPosition=config.len;
    }
    loadMotor0Status(tempStatus);
    // if last status was saved in iddle state 
    // we can replace position, otherwise we have to keep initial state
    if(tempStatus.runStatus==IDDLE)
    {
      Serial.print("Loaded position M1 ");
      Serial.println(tempStatus.lastPosition);
      motorStatus1.lastPosition=tempStatus.lastPosition;
      motorStatus1.pendingChange=false;
      count=motorStatus1.lastPosition;
    }
  }
  // motor 2 last known position

  motorStatus2.runStatus=IDDLE;
  if(rstCount<2)
  {
      // start after power resume, loading position from file  
      Serial.println("Motor 2 status from file");
      loadMotorStatus(motor1stat, motorStatus2);
      count2=motorStatus1.lastPosition;
      // save to prefs to avoid crashes impact for 
      saveMotor0Status(motorStatus2);
  }
  else
  {
      if(digitalRead(END_STOP2)==LOW)
      {
        motorStatus2.lastPosition=0;
      }
      else 
      {
        motorStatus2.lastPosition=config.len2;
      }
      loadMotor1Status(tempStatus);
      if(tempStatus.runStatus==IDDLE)
      {
        motorStatus2.lastPosition=tempStatus.lastPosition;
        count2=motorStatus2.lastPosition;
        Serial.print("Motor 2 position ");
        Serial.println(count2);
      }
  }
   
   rotary.setSensitivity(200);
   rotary2.setSensitivity(200);
   
   // encoder init end

   // start motor
    digitalWrite(LED_BUILTIN, LOW);
  /* digitalWrite(MOT_PINA,LOW);
   digitalWrite(MOT_PINB,HIGH); */
   // pos_pid.begin(); 
   // pos_pid.tune(1, 2.2, 0);  
   // now full setup
   
   rollerControl.setMotorUpCb(motorUp);
   rollerControl.setMotorDownCb(motorDown);
   rollerControl.setMotorStopCb(stopMotor);
   rollerControl.setGetCounterCb(getCounter);
   rollerControl.setTopEndstopCb(getTopEndStop);
   rollerControl.setPercentMqttCb(publishPercent0);
   rollerControl.setMaxPwm(config.maxpwm);
   rollerControl.setMaxRange(config.len);
   rollerControl.init();

   rollerControl1.setMotorUpCb(motorUp2);
   rollerControl1.setMotorDownCb(motorDown2);
   rollerControl1.setMotorStopCb(stopMotor2);
   rollerControl1.setGetCounterCb(getCounter2);
   rollerControl1.setTopEndstopCb(getTopEndStop2);
   rollerControl1.setPercentMqttCb(publishPercent1);
   rollerControl1.setMaxPwm(config.maxpwm);
   rollerControl1.setMaxRange(config.len2);
   rollerControl1.init();
   attachInterrupt(digitalPinToInterrupt(ENC_PINA), rotaryInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINB), rotaryInterrupt, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC2_PINA), rotaryInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_PINB), rotaryInterrupt2, CHANGE);
  
   debouncer.attachClick(handleClickA);
   debouncer2.attachClick(handleClickB);
   wifiConnect();

  publishPerc(0,map(count,0, config.len, 0,100));
  publishPerc(1,map(count2,0, config.len2, 0,100));

   
}

void loop() {
  // OTA Handler
   debouncer.tick();
   debouncer2.tick();
  ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  rollerControl.rollerLoop();
  rollerControl1.rollerLoop();
  // anything to be stored
  if(motorStatus1.pendingChange||motorStatus2.pendingChange)
  {
    if(rollerControl.isIddle()&&rollerControl1.isIddle())
    {
        if(motorStatus1.pendingChange)
         {
           saveMotor0Status(motorStatus1);
           saveMotorStatus(motor0stat,motorStatus1);
           motorStatus1.pendingChange=false;
         }
        if(motorStatus2.pendingChange)
         {
           saveMotor1Status(motorStatus2);
           saveMotorStatus(motor1stat,motorStatus2);
           motorStatus2.pendingChange=false;
         }
    }
  }
  if((lastUpTime+20000)<millis())
  {
    publishUpTime();
    lastUpTime=millis();
  }
  /* if(motorStatus!=IDDLE)
  {
    motorLoop();
  }
  else 
  {
    // nothing special, button handler is in debouncer module
  }
   */
}

static void handleClickA()
{
    motorStatus1.runStatus=MCLOSING;
    if(rollerControl1.isIddle()&&rollerControl.isIddle())
    {
      saveMotor0Status(motorStatus1);
    }
    motorStatus1.pendingChange=true;
    rollerControl.buttonPress();
 
}

static void handleClickB()
{
    motorStatus2.runStatus=MCLOSING;
    if(rollerControl1.isIddle()&&rollerControl.isIddle())
    {
      saveMotor1Status(motorStatus2);
    }
    motorStatus2.pendingChange=true;
    rollerControl1.buttonPress();
 
}


int32_t getCounter()
{
  return(count);
}

int32_t getCounter2()
{
  return(count2);
}


void publishPercent0(uint8_t percent)
{

  publishPerc(0, percent);
}

void publishPercent1(uint8_t percent)
{

  publishPerc(1, percent);
}


int getTopEndStop()
{
   if(digitalRead(END_STOP)==LOW)
      Serial.print("^");
   return(digitalRead(END_STOP));
}

int getTopEndStop2()
{
   if(digitalRead(END_STOP2)==LOW)
      Serial.print("^");
   return(digitalRead(END_STOP2));
}


void stopMotor(int16_t pwm)
{
     Serial.print("S");
      digitalWrite(MOT_PINB,LOW);
      digitalWrite(MOT_PINB,LOW);      
      
      ledcWrite(motTimerChannelA, 0); 
      ledcWrite(motTimerChannelB, 0); 
      motorStatus1.runStatus=IDDLE;
      motorStatus1.lastPosition=count;

      // saveConfig(filename, config);
      // saveMotor0Status(motorStatus1);
}

void stopMotor2(int16_t pwm)
{
     Serial.print("S2 ");
      digitalWrite(MOT2_PINA,LOW);
      digitalWrite(MOT2_PINB,LOW);      
      
      ledcWrite(motTimerChannel2A, 0); 
      ledcWrite(motTimerChannel2B, 0); 
      motorStatus2.runStatus=IDDLE;
      motorStatus2.lastPosition=count2;
      // motor status is saved in async mode 
      // saveConfig(filename, config);
      // saveMotor1Status(motorStatus2);
      

}


void motorDown(int16_t pwm)
{
   Serial.print("D");
   ledcAttachPin(MOT_PINA, motTimerChannelA);
   ledcAttachPin(MOT_PINB, motTimerChannelB);
   ledcWrite(motTimerChannelA, 0); 
   ledcWrite(motTimerChannelB, pwm); 
   // digitalWrite(MOT_PINA,LOW);
   // digitalWrite(MOT_PINB,HIGH); 
   // analogWrite(MOT_PWM,pwm);
   
}

void motorDown2(int16_t pwm)
{
   Serial.print("D2 ");
    ledcAttachPin(MOT2_PINA, motTimerChannel2A);
   ledcAttachPin(MOT2_PINB, motTimerChannel2B);
   
   ledcWrite(motTimerChannel2A, 0); 
   ledcWrite(motTimerChannel2B, pwm); 
   // digitalWrite(MOT_PINA,LOW);
   // digitalWrite(MOT_PINB,HIGH); 
   // analogWrite(MOT_PWM,pwm);
   
}


void motorUp(int16_t pwm)
{
   Serial.print("U");
   
   // analogWrite(MOT_PWM,pwm);
   // motTimerChannel
   ledcAttachPin(MOT_PINA, motTimerChannelA);
   ledcAttachPin(MOT_PINB, motTimerChannelB);
   ledcWrite(motTimerChannelB, 0); 
   ledcWrite(motTimerChannelA, pwm); 
   // digitalWrite(MOT_PINA,HIGH);
   
   
}

void motorUp2(int16_t pwm)
{
   Serial.print("U2 ");
   Serial.println(pwm);
   
   // analogWrite(MOT_PWM,pwm);
   // motTimerChannel
   // ledcAttachPin(MOT2_PINA, motTimerChannel2A);
   // ledcAttachPin(MOT2_PINB, motTimerChannel2B);
   // ledcDetachPin(MOT2_PINA);
   // digitalWrite(MOT2_PINA,HIGH);
   ledcWrite(motTimerChannel2B, 0); 
   ledcWrite(motTimerChannel2A, pwm); 
   // digitalWrite(MOT_PINA,HIGH);
   
   
}

void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  // File file = SPIFFS.open(filename,"r");
  File file = SPIFFS.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));
  Serial.print("Loaded ssid");
  const char* ssid = doc["ssid"];
  Serial.println(ssid);
  // Copy values from the JsonDocument to the Config
  config.len = doc["len"] | 2900;
  config.len2 = doc["len2"] | 2900;
  config.roomNum = doc["roomNum"] | 99;
  Serial.print("Room num ");
  Serial.println(config.roomNum );
  config.maxpwm=doc["maxpwm"] | 200;
  config.minpwm=doc["minpwm"] | 128;
  mqttServer[0]=doc["mqtt"][0] | 192;
  mqttServer[1]=doc["mqtt"][1] | 168;
  mqttServer[2]=doc["mqtt"][2] | 1;
  mqttServer[3]=doc["mqtt"][3] | 1;
  
  
  strlcpy(config.ssid,                  // <- destination
          doc["ssid"] | "xxxx",  // <- source
          sizeof(config.ssid));         // <- destination's capacity

  strlcpy(config.pwd,                  // <- destination
          doc["pwd"] | "yyyy",  // <- source
          sizeof(config.pwd));         // <- destination's capacity


  strlcpy(config.host,                  // <- destination
          doc["host"] | "espRoller1",  // <- source
          sizeof(config.host));         // <- destination's capacity

  
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}

void saveConfig(const char *filename, Config &config) {
 
  // Delete existing file, otherwise the configuration is appended to the file
  // SD.remove(filename);

  // Open file for writing
  /*File file = LittleFS.open(filename,"w");
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  */
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Set the values in the document
  doc["ssid"] = config.ssid;
  doc["pwd"] = config.pwd;
  doc["host"] = config.host;
  doc["roomNum"] = config.roomNum;
  doc["len"]= config.len;
  doc["len2"]= config.len;
  doc["maxpwm"] = config.maxpwm;
  doc["minpwm"] = config.minpwm;
  doc["mqtt"][0]= mqttServer[0];
  doc["mqtt"][1]= mqttServer[1];
  doc["mqtt"][2]= mqttServer[2];
  doc["mqtt"][3]= mqttServer[3];
  serializeJson(doc, Serial);
  //doc.prettyPrint(Serial);
  // Serialize JSON to file
  /*if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file */
 // file.close();


}

void loadMotorStatus(const char *filename, MotorStatus &config) {
  // Open file for reading
  // File file = SPIFFS.open(filename,"r");
  if(!SPIFFS.exists(filename))
  {
      Serial.println("File not found");
      saveMotorStatus(filename, config);
  }
  File file = SPIFFS.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));
  Serial.print("Loaded ssid");
  const char* ssid = doc["run"];
  Serial.println(ssid);
  // Copy values from the JsonDocument to the Config
  config.runStatus = doc["runStatus"] | IDDLE;
  config.lastPosition = doc["lastPosition"] | 99;

  file.close();
}


void saveMotorStatus(const char *filename, MotorStatus &config) {

  if(SPIFFS.exists(filename))
  {
      Serial.print("Deleting file ");
      Serial.println(filename);
      SPIFFS.remove(filename);
  }
  File file = SPIFFS.open(filename, FILE_WRITE);

   StaticJsonDocument<512> doc;

  // Set the values in the document
  doc["runStatus"] = config.runStatus;
  doc["lastPosition"] = config.lastPosition;


  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  file.close();
  serializeJson(doc, Serial);
}
 

void loadMotor0Status(MotorStatus &config) {
  // Open file for reading
  // File file = SPIFFS.open(filename,"r");
  Serial.print("Loading Prefs");
  preferences.begin("roller32", false); 
  int8_t prefExists=preferences.getChar("motor0exists", 0);
  if(prefExists==0)
  {
      saveMotor0Status( config);
  }
  config.runStatus=preferences.getChar("m0S", IDDLE);
  config.lastPosition=preferences.getLong("m0P",0);
  Serial.print("Loaded position ");
  Serial.println(config.lastPosition);
preferences.end();
}


void saveMotor0Status(MotorStatus &config) {
  preferences.begin("roller32", false); 
  Serial.println("Saving Prefs");
  preferences.putChar("motor0exists",1);
  preferences.putChar("m0S", config.runStatus);
  preferences.putLong("m0P", config.lastPosition);
  Serial.print("Saving position ");
  Serial.println(config.lastPosition);
  preferences.end();
}

void loadMotor1Status(MotorStatus &config) {
  // Open file for reading
  // File file = SPIFFS.open(filename,"r");
  preferences.begin("roller32", false); 
  Serial.print("Loading Prefs");
  int8_t prefExists=preferences.getChar("motor1exists", 0);
  if(prefExists==0)
  {
      saveMotor1Status( config);
  }
  config.runStatus=preferences.getChar("m1S", IDDLE);
  config.lastPosition=preferences.getLong("m1P",0);
  Serial.print("Loaded position ");
  Serial.println(config.lastPosition);
preferences.end();
}


void saveMotor1Status(MotorStatus &config) {
  preferences.begin("roller32", false); 
  Serial.println("Saving Prefs");
  preferences.putChar("motor1exists",1);
  preferences.putChar("m1S", config.runStatus);
  preferences.putLong("m1P", config.lastPosition);
  Serial.print("Saving position ");
  Serial.println(config.lastPosition);
  preferences.end();

 
  
}




void wifiConnect()
{
  // OTA SETUP
  WiFi.disconnect(true);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  WiFi.begin(config.ssid, config.pwd);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("espRoller1");
  ArduinoOTA.setHostname(config.host);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // OTA Setup end
 /* pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  */
  reconnect();
}


void reconnect() {
  // Loop until we're reconnected
  Serial.print("MQTT Srv: ");
  Serial.println(mqttServer);
  client.setServer(mqttServer, 1883);
  client.setCallback(mqttCallback);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      
      // ... and resubscribe
      
      // client.subscribe(topicListen);
      snprintf (topic, MSG_TOPIC_SIZE, "digitalcomm/%ld/#", config.roomNum);
      
      client.subscribe(topic);
      
      //topicListen
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}





void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char delim[2]="/";
  char param[MSG_BUFFER_SIZE]={0};
  Serial.println(getParamChar(topic,delim,2,param,false));
  char cPayload[512]={0};  // copy of payload in char version
  for (unsigned int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    cPayload[i]=payload[i];
  }
  Serial.println();
  if(strncmp("setPosition",(const char*) param,12)==0)
   {
     uint16_t currentPosition=atoi((const char*) cPayload);
     uint8_t tokenCount=countTok(topic, delim);
     int8_t intPort=getParam(topic,delim,3,0,false);
     bool stopRequest=false;
     Serial.print("Position ");
     Serial.println(currentPosition);
     Serial.print("Port ");
     Serial.println(intPort);
     if(currentPosition==0)
     {
         if(strncmp("STOP", (const char *) cPayload,4)==0)
            stopRequest=true;
     }
     // motorToPosition(currentPosition);
     switch(intPort)
     {
      case 0:
       if(rollerControl.isIddle())
       {
          motorStatus1.runStatus=MCLOSING;
          if(rollerControl1.isIddle())
          {
            // preferences saved only when second motor is not running 
            saveMotor0Status(motorStatus1);
          }
          motorStatus1.pendingChange=true;
          rollerControl.gotoPercentage(currentPosition);
          
       }
       else
       {
          if(stopRequest)
          {
             rollerControl.stopNow();
          }
          else 
            Serial.println("Ignoring due running motor");
       }
       break;
      case 1:
       if(rollerControl1.isIddle())
       {
          motorStatus2.runStatus=MCLOSING;
          if(rollerControl.isIddle())
          {
             // save preferences only in case, 2nd motor is not running
            saveMotor1Status(motorStatus2);
          }
          motorStatus2.pendingChange=true;
          rollerControl1.gotoPercentage(currentPosition);
       }
       else
       {
          if(stopRequest)
          {
              rollerControl1.stopNow();
              Serial.println("Forced STOP|");
          }
           else
           
            Serial.println("Ignoring due running motor");
       }
       break;
       case 2:
          if(rollerControl1.isIddle()&&rollerControl.isIddle())
          {
               motorStatus2.runStatus=MCLOSING;
               motorStatus2.pendingChange=true;
               motorStatus1.runStatus=MCLOSING;
               motorStatus1.pendingChange=true;
               saveMotor1Status(motorStatus2);
               saveMotor0Status(motorStatus1);
               rollerControl.gotoPercentage(currentPosition);
               rollerControl1.gotoPercentage(currentPosition);

          }
          else
          {
              if(stopRequest)
              {
                 rollerControl.stopNow();
                 rollerControl.stopNow();
              }
          }
       
     }
    //  digitalRooms.setOpenedState(intRoom, intPort, currentPosition);
    //  digitalRooms.queueDigitalPort(intRoom, intPort, PORT_OPEN, 254, NetworkModule::currentRtc);
   }
   else if(strncmp("forcePos",(const char*) param,8)==0)
   {
     int32_t currentPosition=atol((const char*) cPayload);
     uint8_t tokenCount=countTok(topic, delim);
     int8_t intPort=getParam(topic,delim,3,0,false);
     Serial.print("Position ");
     Serial.println(currentPosition);
     Serial.print("Port ");
     Serial.println(intPort);
     // motorToPosition(currentPosition);
      // motorStatus1.runStatus=MCLOSING;
     // rollerControl.gotoPercentage(currentPosition);
     // saveMotor0Status(motorStatus1);
     switch(intPort)
     {
      case 0:
        count=currentPosition;
        motorStatus1.lastPosition=count;
        publishPerc(0,map(count, 0,config.len, 0,100));
        break;
      case 1:
        count2=currentPosition;
        motorStatus2.lastPosition=count2;
        publishPerc(1,map(count2, 0,config.len2, 0,100));
        break;
     }
    //  digitalRooms.setOpenedState(intRoom, intPort, currentPosition);
    //  digitalRooms.queueDigitalPort(intRoom, intPort, PORT_OPEN, 254, NetworkModule::currentRtc);
   }
   else if(strncmp("portOn",(const char*) param,6)==0)
   {
     uint16_t portRequest=atoi((const char*) cPayload);
     uint8_t tokenCount=countTok(topic, delim);
     int8_t intPort=getParam(topic,delim,3,0,false);
     if(portRequest==1)
     {
        digitalWrite(LIGHT_PIN, HIGH);
        Serial.print("LIGHT ON");
     }
     else
     {
        digitalWrite(LIGHT_PIN, LOW);
        Serial.print("LIGHT OFF");
     }
     
   }
  
}

void publishUpTime()
{
     long secs=millis() /1000;

      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/uptime", config.roomNum);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d",secs);
      client.publish(topic,msg);      

      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/restarts", config.roomNum);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d",rstCount);
      client.publish(topic,msg);      

      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/count", config.roomNum, 0);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d",count);
      client.publish(topic,msg);      

      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/count", config.roomNum, 1);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d",count2);
      client.publish(topic,msg);      

      MotorStatus tempStatus;

      loadMotor0Status(tempStatus);
      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/prefcount", config.roomNum, 0);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d", tempStatus.lastPosition);
      client.publish(topic,msg);      

      loadMotor1Status(tempStatus);
      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/prefcount", config.roomNum, 1);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d", tempStatus.lastPosition);
      client.publish(topic,msg);      

      loadMotorStatus(motor0stat, tempStatus);
      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/fcount", config.roomNum, 0);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d", tempStatus.lastPosition);
      client.publish(topic,msg);      

      loadMotorStatus(motor1stat, tempStatus);
      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/fcount", config.roomNum, 1);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d", tempStatus.lastPosition);
      client.publish(topic,msg);      

      // publish light status
       snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%ld/state", config.roomNum, 4);
       if(digitalRead(LIGHT_PIN)==HIGH)
        snprintf (msg, MSG_BUFFER_SIZE, "%1d", 1);
       else 
        snprintf (msg, MSG_BUFFER_SIZE, "%1d", 0);
      client.publish(topic,msg);      
}


void publishPerc(uint8_t port, uint8_t proc)
{
    // int proc=map(steps, 0, config.len, 0,100);
      snprintf (topic, MSG_TOPIC_SIZE, "digitalstate/%ld/%1d/opened", config.roomNum, port);
      snprintf (msg, MSG_BUFFER_SIZE, "%1d",proc);
      client.publish(topic,msg);      
 //      Serial.println(topic);
 //      Serial.println(msg);

}

char* getParamChar(char *req, char *delim, uint8_t sequence,  char* defaultVal, bool stripFirst)
{
   uint8_t counter=0;
   char *token=new char;
   char request[50]={0};
   strncpy(request,req,50);
   char* retVal=defaultVal;
   token=strtok(request,delim);
   while(token!=NULL)
   {
      if(counter==sequence)
        break;

      token=strtok(NULL,delim);
      counter++;
   }
   if(stripFirst)
    token++;
    // copy found string to return value
    memcpy(defaultVal,token, MSG_BUFFER_SIZE);

   // Serial.println(token);
     retVal=token;
   // atp
   return retVal;
}

uint8_t countTok(char *req, char *delim)
{
  uint8_t counter=0;
  char *token;
   char request[50]={0};
   strncpy(request,req,50);

  token=strtok(request,delim);
  while(token!=NULL)
  {
    token=strtok(NULL,delim);
    counter++;
  }
  return counter;

}
/**
 *  method will obtain argument within slash format
 *  like    room/port/time/state
 *  sequence  0   1     2   3
 *  will return param value
 */

uint16_t getParam(char *req, char *delim, uint8_t sequence,  uint16_t defaultVal, bool stripFirst)
{
   uint8_t counter=0;
   char *token;
   char request[50]={0};

   strncpy(request,req,50);

   uint16_t retVal=defaultVal;
   token=strtok(request,delim);
   while(token!=NULL)
   {
      if(counter==sequence)
        break;

      token=strtok(NULL,delim);

      counter++;
   }
   if(stripFirst)
    token++;
   // now translate to number
   if(strncmp("true", token, 4)==0)
   {
     retVal=1;

   }else if(strncmp("false", token, 5)==0)
   {
     retVal=0;

   }else
   {
     retVal=atoi(token);
   }
     // atp
   return retVal;
}
