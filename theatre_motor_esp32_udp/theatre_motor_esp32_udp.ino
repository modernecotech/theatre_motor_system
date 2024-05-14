#include <WiFi.h>
#include "esp32-hal-ledc.h"
#include "AsyncUDP.h"

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

// motor number
String motor = "M01";

// refresh times to check for wifi connection
unsigned long previousMillis = 0;
unsigned long interval = 30000;

AsyncUDP udp;

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(motor.c_str()); //use the motor name as hostname
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}


// pin connections on the arduino for the motors
// M1 (large motor)
const int PWM = 23;
const int DIR = 22;

// PWM properties 
// setting PWM properties. 5khz with 8bit (0-255 speed)
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;

void setup() {
  // set the pinmode, this runs once on boot:
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM, pwmChannel);
  pinMode(DIR, OUTPUT);
  
  // default all to low
  ledcWrite(pwmChannel, 0);
  digitalWrite(DIR, LOW);
  
  // start the serial port listener
  Serial.begin(115200);
  initWiFi();
  delay(1000);
  udp.listen(8080);
  udp.onPacket([](AsyncUDPPacket packet) {
      String signal = String((char*) packet.data());
      Serial.print(signal);
      String direction = signal.substring(3,4);
      Serial.print(direction);
      String sp = signal.substring(4);
      int speed = sp.toInt();
      if (signal.startsWith(motor)) {
        ledcWrite(pwmChannel, speed);
        if (direction == "U") {
          digitalWrite(DIR, HIGH);
        }
        else if (direction == "D") {
          digitalWrite(DIR, LOW);
        }
        else {
          digitalWrite(DIR, LOW);
        }
      }
      else if (signal.startsWith("STOP")) {
        ledcWrite(PWM, 0);
        digitalWrite(DIR, LOW);
      }
    });
  }


void loop() {
  unsigned long currentMillis = millis();
  // put your main code here, to run repeatedly:
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
   
}
