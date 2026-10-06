#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> 


typedef struct struct_message {
  int xVal;
  int yVal;
  int zVal;

  int xVal2;
  int yVal2;
  int zVal2;
  
  int soilstate;
  int BMEstate;
} struct_message;

struct_message jsData;



// Telemetry packet we send back to the handheld
typedef struct {
  float     tempF;
  float     rh;
  float     pressure_hPa;
  int       soil_pct;
  bool      BME_on;
  bool      soil_on;

} rover_to_controller;



uint8_t senderMac[] = {0xA0, 0xA3, 0xB3, 0x80, 0x41, 0xC0}; // <-- replace with SENDER's STA MAC
esp_now_peer_info_t replyPeer;

Servo servo1;
Servo servo2;
Servo servo3;

int servo1_pin = 15;
int servo2_pin = 26;
int servo3_pin = 2;

int AIN1 = 25;    
int AIN2 = 12; 
int BIN1 = 33;
int BIN2 = 32;

// 2nd Motor directional pins
int AAIN1 = 19;
int AAIN2 = 14;
int BBIN1 = 18;
int BBIN2 = 4;

// REMOVE DELAY
// millis() = current time in ms since boot (aka counts over time after boot)
// rover_t = time last ran for the rover update
// rover_dt = how often the updates should run in ms
const uint8_t arm_dt = 20; //servo updates every 8 milliseconds
const uint8_t rover_dt = 10; //motor updates every 5 milliseconds
static unsigned long arm_t = 0; //0 milliseconds
static unsigned long rover_t = 0; //0 milliseconds

const unsigned long soil_dt = 1000; // uint8 only for 0 - 255 ms
static unsigned long soil_t = 0;


const unsigned long BME_dt = 1000;
static unsigned long BME_t = 0;

//Environmental sensors

//soil sensor
int soilsensorpin = 34; // originally 14, but GPIOs 32-39 alwasy work with wifi but GPIOS 0, 2,4, 12-15, 25-27 conflicts with WIFI/ESPNOW
const int AirValue = 3200; // air reading
const int WaterValue = 2700; // wet reading
static bool soil_on = false;


//BME sensor
#define SDA_PIN 21 // SDi
#define SCL_PIN 22 // SCK
Adafruit_BME680 bme;
 
static bool BME_on = false;

int   last_soil   = 0;
float last_tempF  = 0.0f;
float last_rh     = 0.0f;
float last_p_hPa  = 0.0f;

const unsigned long etpacket_dt = 500;
unsigned long etpacket_t = 0;


void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  

  esp_now_register_recv_cb(OnDataRecv);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);

  servo1.attach(servo1_pin, 700, 2400);
  servo2.attach(servo2_pin, 1000, 1900);
  servo3.attach(servo3_pin, 1200, 1900);

  // Initialize Serial Monitor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AAIN1, OUTPUT);
  pinMode(AAIN2, OUTPUT);
  pinMode(BBIN1, OUTPUT);
  pinMode(BBIN2, OUTPUT);

    
   //configure motor PWM functionalitites
  //environment sensors

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    Serial.println("BME680 not found (check SDA/SCL, power, address)");
    BME_on = false;
  } 

  // reasonable oversampling + filtering for smooth readings
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  // disable gas 
  bme.setGasHeater(0, 0);


  memset(&replyPeer, 0, sizeof(replyPeer));
  memcpy(replyPeer.peer_addr, senderMac, 6);
  replyPeer.channel = 0;
  replyPeer.encrypt = false;
  if (!esp_now_is_peer_exist(senderMac)) {
    esp_now_add_peer(&replyPeer);
  }
}


void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&jsData, incomingData, sizeof(jsData));

  
  Serial.print("X: ");
  Serial.print(jsData.xVal);

  Serial.print(",  Y: ");
  Serial.print(jsData.yVal);

  Serial.print(",  Z: ");
  Serial.print(jsData.zVal);

  Serial.print(", X2: ");
  Serial.print(jsData.xVal2);

  Serial.print(",  Y2: ");
  Serial.print(jsData.yVal2);

  Serial.print(",  Z2: ");
  Serial.println(jsData.zVal2);

  
}

void loop() {

  //int pos = 0;
  //int pos2 = 178;

  static int angle = 0;
  static int angle2 = 0;
  static float angle3 = 0;
 
  // for tilt
  static bool updirection = true;


  // move the rover
   //<---- IMPORTANt, prevents rover motors from moving when z button is held 
 // if(jsData.zVal==HIGH ){
  // 1) TOP diagonals first -> turns
    if (millis() - rover_t >= rover_dt) {
      rover_t += rover_dt;
      if (jsData.xVal > 3500 && jsData.yVal < 1000) {
        // turn right
        digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,LOW ); digitalWrite(BIN2,HIGH);
        digitalWrite(AAIN1,HIGH);digitalWrite(AAIN2,LOW);
        digitalWrite(BBIN1,LOW );digitalWrite(BBIN2,HIGH);
      }
      else if (jsData.xVal < 1000 && jsData.yVal < 1000) {
        // turn left
        digitalWrite(AIN1,LOW ); digitalWrite(AIN2,HIGH);
        digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW );
        digitalWrite(AAIN1,LOW );digitalWrite(AAIN2,HIGH);
        digitalWrite(BBIN1,HIGH);digitalWrite(BBIN2,LOW );
      }

      // 2) Straight forward/back next
      else if (jsData.yVal < 1000) {
        // forward (all CW)
        digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);
        digitalWrite(AAIN1,HIGH);digitalWrite(AAIN2,LOW);
        digitalWrite(BBIN1,HIGH);digitalWrite(BBIN2,LOW);
      }
      else if (jsData.yVal > 3500) {
        // backward (all CCW)
        digitalWrite(AIN1,LOW ); digitalWrite(AIN2,HIGH);
        digitalWrite(BIN1,LOW ); digitalWrite(BIN2,HIGH);
        digitalWrite(AAIN1,LOW );digitalWrite(AAIN2,HIGH);
        digitalWrite(BBIN1,LOW );digitalWrite(BBIN2,HIGH);
      }

      // 3) Strafes last (will not steal forward/back)
      else if (jsData.xVal > 3500) {
        // strafe right
        digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,LOW ); digitalWrite(BIN2,HIGH);
        digitalWrite(AAIN1,LOW );digitalWrite(AAIN2,HIGH);
        digitalWrite(BBIN1,HIGH);digitalWrite(BBIN2,LOW);
      }
      else if (jsData.xVal < 1000) {
        // strafe left
        digitalWrite(AIN1,LOW ); digitalWrite(AIN2,HIGH);
        digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW );
        digitalWrite(AAIN1,HIGH);digitalWrite(AAIN2,LOW );
        digitalWrite(BBIN1,LOW );digitalWrite(BBIN2,HIGH);
      }
      else {
      // not in rover mode or safety held -> stop
        digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
        digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
        digitalWrite(AAIN1,LOW);digitalWrite(AAIN2,LOW);
        digitalWrite(BBIN1,LOW);digitalWrite(BBIN2,LOW);
      }
    }
 // }
  
  

  // move the arm
  if (millis() - arm_t >= arm_dt) {
    arm_t += arm_dt;
    //FOR LIFT

    //up
    // && jsData.xVal2 > 1500 && jsData.xVal2 < 3600
    if (jsData.yVal2 > 3500) {
      if (angle2 > 0) {
      angle2 -= 2;
      angle2 = constrain(angle2, 0, 178);
      servo2.write(angle2);
      }

    //down
    // && jsData.xVal2 > 1500 && jsData.xVal2 < 3000
    } else if (jsData.yVal2 < 1500) { // doesn't read diagonal
        if (angle2 < 180) {
          angle2 += 2;
          angle2 = constrain(angle2, 0, 178);
          servo2.write(angle2);
        }
    //FOR gripper
    //open // && jsData.yVal2 > 1500 && jsData.yVal2 < 3000
    } else if (jsData.xVal2 > 3500) {
        if (angle < 180) {
          angle += 3;
          angle = constrain(angle, 0, 178);
          servo1.write(angle);
        }
    //close // && jsData.yVal2 > 1500 && jsData.yVal2 < 3000
    } else if (jsData.xVal2 < 1500) {  
        if (angle > 0) {
          angle -= 3;
          angle = constrain(angle, 0, 178);
          servo1.write(angle);
        }
       //for tilt
    } else if (jsData.zVal2 == 0) {
      if (updirection == true) {
          angle3 += 2;
          if (angle3 > 140) {
            angle3 = 140;
            updirection = false;
          }
        } else {
            angle3 -= 2;
            if (angle3 <= 0){
              angle3 = 0;
              updirection = true;
          }
        }
        servo3.write(angle3);
      }
  }
    

    

  // soil sensor
  
  if (millis() - soil_t >= soil_dt) {
    soil_t += soil_dt;
    
    /*if (jsData.soilstate == true && soil_on == false){
      soil_on = true;
    } else if (jsData.BMEstate == false) {
      soil_on = false;
    }*/

    soil_on = jsData.soilstate;        // UI flag only (still sent back)

    int sensorValue = analogRead(soilsensorpin);
    int moisturePercentage = map(sensorValue, AirValue, WaterValue, 0, 100);
    last_soil = constrain(moisturePercentage, 0, 100); // Keep within 0-100%

      /*
        Serial.print("Soil Moisture: ");
        Serial.print(soilMoistureValue);
        Serial.print("    ||  Moisture Percentage of Sensor: ");
        Serial.print(soilmoisturepercent);
        Serial.println("%");
      */
    
  }



  if (millis() - BME_t >= BME_dt){
    BME_t += BME_dt;
    if (jsData.BMEstate == true && BME_on == false){
      BME_on = true;
    } else if (jsData.BMEstate == false) {
      BME_on = false;
    }

    if (BME_on == true && bme.performReading()) {
      float tempC = bme.temperature;
      last_tempF  = (tempC * 9.0f / 5.0f) + 32.0f;
      last_rh     = bme.humidity;
      last_p_hPa  = bme.pressure / 100.0f;
      
      
      
      /*float tempC = bme.temperature;        // °C
      float rh    = bme.humidity;           // humidity %
      float p_hPa = bme.pressure / 100.0;   // hPa (atmospheric/station pressure)
      float tempF = (tempC * 9.0 / 5.0 ) + 32; */

      /*
      Serial.print("Temperature = "); 
      Serial.print(tempF);   // temp
      Serial.print("°F  ||  ");
      Serial.print("Humidity= "); 
      Serial.print(rh);     // humidity
      Serial.print("%  ||  ");
      Serial.print("Atmospheric Pressure = "); 
      Serial.print(p_hPa);  // atmospheric pressure
      Serial.println(" hPa"); */
  
    }
  } 

  if (millis() - etpacket_t >= etpacket_dt) {
    etpacket_t += etpacket_dt;
 
    rover_to_controller etpacket = {
      last_tempF,      // tempF
      last_rh,         // rh
      last_p_hPa,      // pressure_hPa
      last_soil,       // soil_pct
      BME_on,
      soil_on,
    };
      
    esp_now_send(senderMac, (uint8_t*)&etpacket, sizeof(etpacket));
  }
}

