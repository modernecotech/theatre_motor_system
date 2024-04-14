#include <MIDI.h>
#include <AppleMIDI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "esp32-hal-ledc.h"
#include <ezButton.h>

// use arduino upesy ESP32 WROOM devkit as device.

// motor number
const String motor = "1";
// Replace with your network credentials
const char* ssid = "";
const char* password = "";


// refresh times to check for wifi connection
unsigned long previousMillis = 0;
unsigned long interval = 30000;

//rotation of the motor
unsigned int rotationCounter = 0;
bool on_state = false;

// limit switch. Pin C to ground and pin NO (normally open) to arduino pin. HIGH when touched, LOW when untouched.

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
// M1 (large motor). PWM - speed of motor, DIR - direction of motor high up, low down), HAL - HAL effect magnetometer for rotation detection
const int PWM = 23;
const int DIR = 22;
const int hall_pin = 21;

ezButton limitSwitch(35);  // create ezButton object that attach to pin GPIO25

// PWM properties 
// setting PWM properties. 5khz with 8bit (0-255 speed)
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;


APPLEMIDI_CREATE_DEFAULTSESSION_INSTANCE(); 

void MyMidiCallback(uint8_t channel, uint8_t note, uint8_t velocity)
{
    // Do something with the received MIDI message here
    Serial.print(channel);
    Serial.print(note);
    Serial.println(velocity);

    // motor up at speed 100 - use "rotationCounter" to manage when it stops
    // ledcWrite(PWM, 100);
    // digitalWrite(DIR, HIGH);
    

    // motor DOWN at speed 100 - use "rotationCounter" to manage when it stops
    // ledcWrite(PWM, 100);
    // digitalWrite(DIR, LOW);
    
}

void setup() {
  // set the pinmode, this runs once on boot:
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM, pwmChannel);
  pinMode(DIR, OUTPUT);
  pinMode(hall_pin, INPUT);

  // default all to low
  ledcWrite(pwmChannel, 0); // Motor speed 0 to 255
  digitalWrite(DIR, LOW); // LOW for down and HIGH for motor UP

  limitSwitch.setDebounceTime(50);  // set debounce time to 50 milliseconds

  
  // start the serial port listener
  Serial.begin(115200);
  initWiFi();
  delay(1000);

  // Initialize the AppleMIDI library
  MIDI.begin(); // listens on channel 1   
  MIDI.setHandleNoteOn(MyMidiCallback);
  }


void loop() {
  unsigned long currentMillis = millis();
  limitSwitch.loop();  // MUST call the loop() function first

  // put your main code here, to run repeatedly:
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  MIDI.read();  

  if (limitSwitch.isPressed()) {
    rotationCounter = 0;
    Serial.println("The limit switch is touched");
  }


  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
  if (digitalRead(hall_pin)==0){
    if (on_state==false){
        on_state = true;
        if (digitalRead(DIR)==0){
              rotationCounter+=1;
            } else{
                  rotationCounter-=1;
            }
        Serial.println(rotationCounter);
      }
  } else{
    on_state = false;
  }


}
