// Libraries
#include "Arduino.h"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <Update.h>
//#include "esp_camera.h"

#include "AsyncJson.h"
#include "ArduinoJson.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "SD_MMC.h"            // SD Card ESP32

//brownout dedector
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//variables for create async delay with millis
int period60 = 60000; //ms
unsigned long timeNowRead = 0;


//I2C clock ve data settings for bme280 connection
#define I2C_SDA 13
#define I2C_SCL 15

// sea level pressure: 1013.25 hectopascal
#define SEALEVELPRESSURE_HPA (1013.25)

//bme280 init settings
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;
float temperature, pressure, altitude, humidity;


// ESP32 network
IPAddress staticIP(192, 168, 1, 150);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 1);
IPAddress dns(192, 168, 1, 1);
const char* mySsid = "Gorkem"; 
const char* myPassword = "gorkem07123";

//SQL database server
const char* serverName = "http://192.168.1.24/esp32webserver/query.php?";




// async server
AsyncWebServer server(80);
#define update_username "admin"
#define update_password "secret"
#define UPDATE_PATH "/update"
const char *update_path = UPDATE_PATH;
#define DEBUG_ESP_PORT Serial
String inputMessage1, inputMessage2, inputMessage3, inputMessage4;

//motor and L298n
int angle = 0;
int leftMotor1  = 16;  //left motors forward
int leftMotor2  = 0;  //left motors backward
int rightMotor1 = 14;  //right motors forward
int rightMotor2 = 2;  //right motors backward
int enablePinA = 4;    //enable pinA pwm
int enablePinB = 12;    //enable pinB pwm
//pwm settings
const int freq = 30000;
const int pwmResolution = 8;
//pwm settings for left motors
const int pwmChannel1 = 0;
int leftSpeed = 0;   //dutycycle
//pwm settings for right motors
const int pwmChannel2 = 1;
int rightSpeed = 0;  //dutycycle
int leftSpeedTemp = 0;
int rightSpeedTemp = 0;

//CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22



void setup() {
  
  
  // Serial port başlat
  Serial.begin(115200);
  
   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

   //disable gpio4 internal flashlight
   if(!SD_MMC.begin("/sdcard", true)){
        Serial.println("Card Mount Failed");        
    }

  

//  wifi network setup
  if (!WiFi.config(staticIP, gateway, subnet, dns, dns)) {
    Serial.println("STA Failed to configure");
  }
  
  // start wifi connection
  WiFi.begin(mySsid, myPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  // Local ip address
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.subnetMask());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.dnsIP(0));
  Serial.println(WiFi.dnsIP(1));

  //call bme280 init
  initSensor();

  //l298n pins
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  // pwm setup for left motors
  ledcSetup(pwmChannel1, freq, pwmResolution);
  ledcAttachPin(enablePinA, pwmChannel1);


  // pwm setup for right motors
  ledcSetup(pwmChannel2, freq, pwmResolution);
  ledcAttachPin(enablePinB, pwmChannel2);




  // http get for json data
  server.on("/bme280", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncJsonResponse * response = new AsyncJsonResponse();
    response->addHeader("Server", "ESP Async Web Server");
    JsonObject root = response->getRoot();
    root["temperature"] = temperature;
    root["pressure"] = pressure;
    root["altitude"] = altitude;
    root["humidity"] = humidity;
    response->setLength();
    request->send(response);   
  });

  //OTA programming
  server.on(update_path, HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!request->authenticate(update_username, update_password)) {
      return request->requestAuthentication();
    }
    handleUpdate(request);
  });
  server.on("/doUpdate", HTTP_POST,
  [](AsyncWebServerRequest * request) {},
  [](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!request->authenticate(update_username, update_password)) {
      return request->requestAuthentication();
    }
    handleDoUpdate(request, filename, index, data, len, final);
  });

  
  
  // get request (IP/drive?)
  server.on("/drive", HTTP_POST, [] (AsyncWebServerRequest * request) {   
    // yapılan isteği kontrol et
    if ((request->hasParam("angle"), true) && (request->hasParam("leftSpeed"), true) && (request->hasParam("rightSpeed"), true)) {
      inputMessage1 = request->getParam("angle",true)->value();
      inputMessage2 = request->getParam("leftSpeed",true)->value();
      inputMessage3 =  request->getParam("rightSpeed",true)->value();
      angle = inputMessage1.toInt();
      leftSpeed = inputMessage2.toInt();
      rightSpeed = inputMessage3.toInt();
      
      Serial.println(inputMessage1);
      Serial.println(inputMessage2);
      Serial.println(inputMessage3);
      
    }
   request->send(200, "text/plain", "OK");   
    
   
  });


  // asenkron server başlat
  server.begin();

  readValues();
}

void loop() { 
  
  
  if(altitude == 44330){
    initSensor();
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    drive(0, 0, 0);
  }


  if (millis() >= timeNowRead + period60) {
    timeNowRead += period60;
    readValues();
    sqlServerRequest();   
  }

  if(leftSpeed != leftSpeedTemp || rightSpeed != rightSpeedTemp){
    
    leftSpeedTemp = leftSpeed;
    rightSpeedTemp = rightSpeed;
    drive(angle, leftSpeed, rightSpeed);
  }
  
}

//bme280 initialization
void initSensor()
{
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
  bool bmeStatus = bme.begin(0x76, &I2CBME);
  if (!bmeStatus) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  bme.setSampling( Adafruit_BME280::MODE_FORCED,
                   Adafruit_BME280::SAMPLING_X2,     // temperature
                   Adafruit_BME280::SAMPLING_X4,    // pressure
                   Adafruit_BME280::SAMPLING_X1,     // humidity
                   Adafruit_BME280::FILTER_X4 );
}



void readValues() {
  bme.takeForcedMeasurement();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

}

void sqlServerRequest() {

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    http.addHeader("Content-Type", "application/x-www-form-urlencoded");


    String httpRequestData = "temperature=" + String(temperature) + "&pressure=" + String(pressure)
                             + "&altitude=" + String(altitude) + "&humidity=" + String(humidity) + "&apikey=" + "gorkem" ;

    Serial.println(httpRequestData);
    http.begin(serverName + httpRequestData);
    int httpResponseCode = http.POST(httpRequestData);


    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}

//ota programming functions
//html header
String getHTMLHead() {
  String header = F("<!DOCTYPE html><html lang=\"en\"><head>");
  //header += F("<link href=\"/local.css\" rel=\"stylesheet\">");
  header += F("</head>");
  header += F("<body>");
  return header;
}
//html footer
String getHTMLFoot() {
  return F("</body></html>");
}
//upload button & post file to doUpdate
void handleUpdate(AsyncWebServerRequest *request) {
  String response_message;
  response_message.reserve(1000);
  response_message = getHTMLHead();
  response_message += "<script> \
    function notify_update() {document.getElementById(\"update\").innerHTML = \"<h2>Updating...</h2>\"\; } \
    </script>";
  response_message += "Firmware = *.esp32.bin<br>SPIFFS = *.spiffs.bin<br> \
  <form method='POST' action='/doUpdate' enctype='multipart/form-data' target='_self' onsubmit='notify_update()'> \
  <input type='file' name='update'><br> \
  <input type='submit' value='Do update'></form> \
  <div id=\"update\"></div>";
  response_message += getHTMLFoot();
  request->send(200, "text/html", response_message);
};
//
void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    size_t content_len;
    content_len = request->contentLength();
    // check file names for type
    int cmd = (filename.indexOf(F(".spiffs.bin")) > -1 ) ? U_SPIFFS : U_FLASH;
    if (cmd == U_FLASH && !(filename.indexOf(F("esp32.bin")) > -1) ) return; // wrong image for ESP32
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
      Update.printError(DEBUG_ESP_PORT);
    }
  }
  if (Update.write(data, len) != len) {
    Update.printError(DEBUG_ESP_PORT);
  }
  if (final) {
    if (!Update.end(true)) {
      Update.printError(DEBUG_ESP_PORT);
    } else {
      String response_message;
      response_message.reserve(1000);
      response_message = getHTMLHead();
      response_message += "<h2>Please wait while the device reboots</h2> \
      <meta http-equiv=\"refresh\" content=\"20;url=/\" />";
      response_message += getHTMLFoot();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/html", response_message);
      response->addHeader("Refresh", "20");
      response->addHeader("Location", "/");
      request->send(response);
      delay(100);
      ESP.restart();
    }
  }
}

void drive(int angle, int leftSpeed, int rightSpeed) {
  Serial.println("drive action");  
  ledcWrite(pwmChannel1, leftSpeed);
  ledcWrite(pwmChannel2, rightSpeed);
  //forward
  if (5 < angle && angle < 175) {

    digitalWrite(leftMotor1 , HIGH);        //left motors forward
    digitalWrite(leftMotor2 , LOW);                //left motors backward
    digitalWrite(rightMotor1 , HIGH);      //right motors forward
    digitalWrite(rightMotor2 , LOW);               //right motors backward
  }
  //backward
  else if (185 < angle && angle < 350) {
    digitalWrite(leftMotor1 , LOW);                 //left motors forward
    digitalWrite(leftMotor2 , HIGH);         //left motors backward
    digitalWrite(rightMotor1 , LOW);                //right motors forward
    digitalWrite(rightMotor2 , HIGH);       //right motors backward
  }
  //right
  else if ((0 <= angle && angle < 5) || (355 <= angle && angle < 360)) {


    digitalWrite(leftMotor1 , HIGH);        //left motors forward
    digitalWrite(leftMotor2 , LOW);                //left motors backward
    digitalWrite(rightMotor1 , LOW);               //right motors forward
    digitalWrite(rightMotor2 , HIGH);      //right motors backward
  }
  //left
  else if (175 <= angle && angle <= 185) {
    digitalWrite(leftMotor1 , LOW);               //left motors forward
    digitalWrite(leftMotor2 , HIGH);      //left motors backward
    digitalWrite(rightMotor1 , HIGH);      //right motors forward
    digitalWrite(rightMotor2 , LOW);              //right motors backward
  }
  else {
    digitalWrite(leftMotor1 , LOW);                 //left motors forward
    digitalWrite(leftMotor2 , LOW);                 //left motors backward
    digitalWrite(rightMotor1 , LOW);                //right motors forward
    digitalWrite(rightMotor2 , LOW);                //right motors backward
  } 
}


      
  

  
