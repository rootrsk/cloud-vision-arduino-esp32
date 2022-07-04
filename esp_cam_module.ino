/* Required Headers */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include "Ticker.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "Base64.h"
/* End of Header */

/* ESP_32 Pin Configuration */
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
/* End of pin configuration*/

/* Variable Decleration */
#define LED_BUILTIN 4 //flash light 
#define echoPin 2 // UV echo pin
#define trigPin 13 // UV trigger pin
#define sound  14 //Sound Sensor
#define USE_SERIAL Serial
// #define SERVER   "rootrsk-security-api.herokuapp.com"
#define SERVER "rootrsk-cloudvision.herokuapp.com"
#define PORT 443 // server port
#define BOUNDARY     "--------------------------133747188241686651551404" //from boundary
#define TIMEOUT  20000 //webrequest timeout
void captureImage(String s);
void sendSensorStatus();
WiFiMulti WiFiMulti;
SocketIOclient socketIO;
void timeCapture();
void changeCaptureStatus();
Ticker imageTimer(timeCapture,300000);
Ticker sensorTimer(sendSensorStatus,30000);
Ticker captureStatusTimer(changeCaptureStatus,60000);
const int timerInterval = 60000;    // time between each HTTP POST image
unsigned long previousMillis = 0;  
boolean U = true;
boolean S= true;
boolean F = true;
boolean SHOULD_CAPTURE = true;
String path = "/arduino/upload-image";
String token = "auth_token = "; 
String imageName = "&imageName=rootrsk_esp32_cam.jpg";
String image = "&image=";
/* End of Varaiable decleration */

/* Setup Function  */
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    pinMode (sound,INPUT);
    pinMode (echoPin,INPUT);
    pinMode (trigPin,OUTPUT);
    pinMode (LED_BUILTIN, OUTPUT);
    USE_SERIAL.begin(115200);
    USE_SERIAL.setDebugOutput(true);
    USE_SERIAL.println();
    WiFiMulti.addAP("ROOTRSK", "7909@1234");
    while(WiFiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();
    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());
    
    socketIO.begin("rootrsk-cloudvision.herokuapp.com", 80, "/socket.io/?EIO=4");
    socketIO.onEvent(socketIOEvent);

    /* Camera configuration */
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
   if(psramFound()){
       config.frame_size = FRAMESIZE_SVGA;
       config.jpeg_quality = 12;  //0-63 lower number means higher quality
       config.fb_count = 2;
   } else {
       config.frame_size = FRAMESIZE_CIF;
       config.jpeg_quality = 12;  //0-63 lower number means higher quality
       config.fb_count = 1;
   }
   /* End of camera congiguration */
   
   Serial.println("Initlizing camera with defined configuration");
   esp_err_t err = esp_camera_init(&config);
   if (err != ESP_OK) {
       Serial.printf("Camera init failed with error 0x%x", err);
       delay(1000);
       ESP.restart();
   }
   Serial.println("Everyting is ready to go.");
   imageTimer.start();
   sensorTimer.start();
//   captureStatusTimer.
   

}
/* End of Setup function */
unsigned long messageTimestamp = 0;

/* Loop function */
void loop() {
    socketIO.loop();
    imageTimer.update();
    sensorTimer.update();
    captureStatusTimer.update();
    uint64_t now = millis();
    //* Sensors Readings  */
    Serial.print("Sound->");
    
    int soundReading = digitalRead(sound);
    Serial.println(soundReading);
    digitalWrite(trigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    long duration = pulseIn(echoPin,HIGH);
    int distance = duration * 0.034/2;
    if(distance < 100 && U==true){
      Serial.println("Capturing by motion");
      sensorCapture("motion");
    }
    if(soundReading < 1 && S==true){
      Serial.println("Capturing by sound");
      sensorCapture("sound");
    }
    Serial.print("Distance-->");
    Serial.println(distance);
}


/* End of Loop Function */

/*Any Event which is called from server side can be handled here */
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            USE_SERIAL.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
        {
            char * sptr = NULL;
            int id = strtol((char *)payload, &sptr, 10);
            USE_SERIAL.printf("[IOc] get event: %s id: %d\n", payload, id);
            if(id) {
                payload = (uint8_t *)sptr;
            }
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload, length);
            if(error) {
                USE_SERIAL.print(F("deserializeJson() failed: "));
                USE_SERIAL.println(error.c_str());
                return;
            }

            String eventName = doc[0];
            USE_SERIAL.printf("[IOc] event name: %s\n", eventName.c_str());
            USE_SERIAL.printf("%s",eventName);
            
            if(eventName=="capture-image"){
                captureImage("server");
                break;
            }
            if(eventName=="FF"){
                F=false;
                sendSensorStatus();
                break;
            }
            if(eventName == "OF"){
                F=true;
                sendSensorStatus();
                break;
            }
            if(eventName == "FS"){
                turnOffSs();
                break;
            }
            if(eventName == "OS"){
               turnOnSs();
                break;
            }
            if(eventName == "OU"){
                turnOnUv();
                break;
            }
            if(eventName == "FU"){
                turnOffUv();
                break;
            }
            // Message Includes a ID for a ACK (callback)
            if(id) {
                // creat JSON message for Socket.IO (ack)
                DynamicJsonDocument docOut(1024);
                JsonArray array = docOut.to<JsonArray>();
                // add payload (parameters) for the ack (callback function)
                JsonObject param1 = array.createNestedObject();
                param1["now"] = millis();
                // JSON to String (serializion)
                String output;
                output += id;
                serializeJson(docOut, output);
                // Send event
                socketIO.send(sIOtype_ACK, output);
            }
        }
            break;
        case sIOtype_ACK:
            USE_SERIAL.printf("[IOc] get ack: %u\n", length);
            break;
        case sIOtype_ERROR:
            USE_SERIAL.printf("[IOc] get error: %u\n", length);
            break;
        case sIOtype_BINARY_EVENT:
            USE_SERIAL.printf("[IOc] get binary: %u\n", length);
            break;
        case sIOtype_BINARY_ACK:
            USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
            break;
    }
}
void turnOffUv(){
  Serial.write("FU");
  U= false;
  sendSensorStatus();
}
void turnOnUv(){
  Serial.write("OU");
  U = true;
  sendSensorStatus();
}
void turnOffSs(){
  Serial.write("FS");
  S = false;
  sendSensorStatus();
}
void turnOnSs(){
  Serial.write("OS");
  S = true;
  sendSensorStatus();
}
void timeCapture (){
  captureImage("timer");
}
void changeCaptureStatus (){
  SHOULD_CAPTURE = true;
  captureStatusTimer.stop();
}
void sensorCapture(String s){
  if(SHOULD_CAPTURE == false){
    Serial.println("Camera Not ready");
    return;
  }
  SHOULD_CAPTURE = false;
  captureStatusTimer.start();
  if(s=="sound"){
    captureImage("sound");
  }else{
    captureImage("motion");
  }
}
void captureImage(String s) {
   String getAll;
   String getBody;
   //instance of camera fb t
   Serial.println("Image capture function is called.");
   if(s=="server"){
      sendCameraStatus("server","requested");
   }else if(s=="sound"){
      sendCameraStatus("sound","requested");
   }else if(s=="motion"){
      sendCameraStatus("motion","requested");
   }else{
      sendCameraStatus("timer","requested");
   }
   
   camera_fb_t * fb = NULL;
   //capture image
   if(F==true){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
   }
   fb = esp_camera_fb_get();
   delay(200);
   digitalWrite(LED_BUILTIN, LOW);
   //check if any error occured during capture 
   sendCameraStatus(s,"captured");
   if(!fb) {
       Serial.println("Camera capture failed");
       delay(1000);
       //restart if error occoured
       ESP.restart();
   }
   Serial.println("Image captured successfully");
   Serial.println("Connect to " + String(SERVER));
  
   WiFiClientSecure client_tcp;
   client_tcp.setInsecure();
   sendCameraStatus(s,"uploading");
   if(client_tcp.connect(SERVER, 443)) {
       Serial.println("Connection successful");
       char *input = (char *)fb->buf;
       char output[base64_enc_len(3)];
       String imageFile = "data:image/jpeg;base64,";
       for(int i=0;i<fb->len;i++) {
       base64_encode(output, (input++), 3);
       if (i%3==0) imageFile += urlencode(String(output));
     }
 
       String Data = token+imageName+image;
       Serial.println(imageFile.length());
       esp_camera_fb_return(fb);
       client_tcp.println("POST " + path + " HTTP/1.1");
       client_tcp.println("Host: " + String(SERVER));
       client_tcp.println("Content-Length: " + String(Data.length()+imageFile.length()));
       client_tcp.println("Content-Type: application/x-www-form-urlencoded");
       client_tcp.println("Connection: keep-alive");
       client_tcp.println();
       client_tcp.print(Data);
       int Index;
       for (Index = 0; Index < imageFile.length(); Index = Index+1000) {
          client_tcp.print(imageFile.substring(Index, Index+1000));
       }
       // timeout 10 seconds
       int waitTime = 10000;  
       long startTime = millis();
       boolean state = false;
       while ((startTime + waitTime) > millis()){
       Serial.print(".");
       delay(100);     
       while (client_tcp.available()) {
           char c = client_tcp.read();
           if (state==true) getBody += String(c);       
           if (c == '\n') {
               if (getAll.length()==0) state=true;
               getAll = "";
           }
           else if (c != '\r')
               getAll += String(c);
           startTime = millis();
       }
       if (getBody.length()>0) break;
   }
       client_tcp.stop();
       Serial.println();
       Serial.println(getBody);
       sendCameraStatus(s,"uploaded");
   }else {
       getBody="Connected to " + String(SERVER) + " failed.";
       Serial.println("Connected to " + String(SERVER) + " failed.");
   }
}

void sendSensorStatus(){
    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    array.add("sensor-update");
    JsonObject param1 = array.createNestedObject();
    param1["S"] = S;
    param1["U"] = U;
    param1["F"] = F;
    String output;
    serializeJson(doc, output);
    socketIO.sendEVENT(output);
    USE_SERIAL.println(output);
}
void sendCameraStatus(String by, String sts){
    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    array.add("camera-update");
    JsonObject param1 = array.createNestedObject();
    param1["by"] = by;
    param1["status"] = sts;
    String output;
    serializeJson(doc, output);
    socketIO.sendEVENT(output);
    USE_SERIAL.println(output);
}
 
//url encoded function to convert image to base64(binery to string)
String urlencode(String str){
   String encodedString="";
   char c;
   char code0;
   char code1;
   char code2;
   for (int i =0; i < str.length(); i++){
       c=str.charAt(i);
       if (c == ' '){
           encodedString+= '+';
       } else if (isalnum(c)){
           encodedString+=c;
       } else{
           code1=(c & 0xf)+'0';
           if ((c & 0xf) >9){
               code1=(c & 0xf) - 10 + 'A';
           }
           c=(c>>4)&0xf;
           code0=c+'0';
           if (c > 9){
               code0=c - 10 + 'A';
           }
           code2='\0';
           encodedString+='%';
           encodedString+=code0;
           encodedString+=code1;
           //encodedString+=code2;
       }
       yield();
   }
   return encodedString;
}
