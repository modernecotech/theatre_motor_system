#include <MIDI.h>
#include <AppleMIDI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "esp32-hal-ledc.h"
#include <ezButton.h>
#include <ArduinoMDNS.h>
#include <ESPmDNS.h>

// use arduino upesy ESP32 WROOM devkit as device.

// motor name/number
const String motor = "motor1";
// Replace with your network credentials
const char* ssid = "";
const char* password = "";


// refresh times to check for wifi connection
unsigned long previousMillis = 0;
unsigned long interval = 30000;


// pin connections on the arduino for the motors
// M1 (large motor). PWM - speed of motor, DIR - direction of motor high up, low down), HAL - HAL effect magnetometer for rotation detection
const int PWM = 23;
const int DIR = 22;
const int hall_pin = 21;

// limit switch. Pin C to ground and pin NO (normally open) to arduino pin. HIGH when touched, LOW when untouched.
// Use Switch PIN 32 as it needs a INPUT/OUTPUT capable GPIO port 
const int SWITCH_PIN = 32;

ezButton limitSwitch(SWITCH_PIN);  // create ezButton object

// PWM properties
// setting PWM properties. 5khz with 8bit (0-255 speed)
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;

//rotation of the motor
int rotationCounter = 0;
int rotationSet = -1;
bool on_state = false;
int positionFactor = 1;
int floorPosition = 128;
int MotorSpeed = 100;

APPLEMIDI_CREATE_DEFAULTSESSION_INSTANCE();

//APPLEMIDI_CREATE_INSTANCE(WiFiUDP, MIDI, motor.c_str(), 5004);

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
  Serial.println();
  Serial.print("Connected to SSID:");
  Serial.println(ssid);
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());
}

void MidiProgramChangeCallback(uint8_t channel, uint8_t progNumber)
{
  Serial.print("MIDI: ProgramChange: channel:");
  Serial.print(channel);
  Serial.print(" programNumber:");
  Serial.println(progNumber);
//    cmd_map={'stop':1,'up':2,'down':3,'floor':4}
  switch (progNumber) {
    case 1:
      ledcWrite(pwmChannel, 0); // Stop Motor
      Serial.println("Stopping motor");
      break;
    case 2:
      ledcWrite(pwmChannel, 100);
      digitalWrite(DIR, HIGH);
      Serial.println("Moving motor FORWARD(HIGH)");
      break;
    case 3:
      ledcWrite(pwmChannel, 100);
      digitalWrite(DIR, LOW);
      Serial.println("Moving motor BACK(LOW)");
      break;
    case 4:
      Serial.print("Setting Floor at:");
      Serial.println(rotationCounter);
      floorPosition = rotationCounter;
      break;
    case 5:
      Serial.print("Setting Top(zero) at:");
      Serial.println(rotationCounter);
      rotationCounter = 0;
      break;
    default:
      // statements
      break;
  }
}
  
void MidiNoteOnCallback(uint8_t channel, uint8_t note, uint8_t velocity)
{
  unsigned int motorPosition;
  
  motorPosition = (floorPosition * (127-velocity)) / 127;

  // Do something with the received MIDI message here
  Serial.print("MIDI: NoteOn: channel:");
  Serial.print(channel);
  Serial.print(" note:");
  Serial.print(note);
  Serial.print(" velocity:");
  Serial.print(velocity);
  Serial.print(" Moving to Position:");
  Serial.println(motorPosition);

  if (note==0) {
    MotorSpeed = velocity;
    Serial.print("Setting Motor speed:");
    Serial.println(MotorSpeed);
    if (ledcRead(pwmChannel) > 0) {
      ledcWrite(pwmChannel, MotorSpeed);
    }
    return;
  }

  if (motorPosition < rotationCounter) {
    // motor up at speed 100 - use "rotationCounter" to manage when it stops
    rotationSet = motorPosition;
    ledcWrite(pwmChannel, MotorSpeed);
    digitalWrite(DIR, HIGH);
    Serial.println("Moving motor FORWARD(HIGH)");
  } else if (motorPosition > rotationCounter) {
    // motor DOWN at speed 100 - use "rotationCounter" to manage when it stops
    rotationSet = motorPosition;
    ledcWrite(pwmChannel, MotorSpeed);
    digitalWrite(DIR, LOW);
    Serial.println("Moving motor BACK(LOW)");
  }
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

  pinMode(SWITCH_PIN, INPUT_PULLUP); // Only works on non-input-only GPIO PINS
  limitSwitch.setDebounceTime(50);  // set debounce time to 50 milliseconds

  // start the serial port listener
  Serial.begin(115200);
  initWiFi();
  delay(1000);

  // Initialize the AppleMIDI library
  MIDI.begin(); // listens on channel 1
  MIDI.setHandleNoteOn(MidiNoteOnCallback);
  MIDI.setHandleProgramChange(MidiProgramChangeCallback);
  AppleMIDI.setHandleConnected([](const APPLEMIDI_NAMESPACE::ssrc_t & ssrc, const char* name) {
  Serial.println("AppleMIDI: Connected");
  });

  // Set up mDNS responder
  if (!MDNS.begin(motor.c_str())) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  // Add apple-midi mDNS service so it appears in MIDI browser
  MDNS.addService("_apple-midi", "udp", AppleMIDI.getPort());
  Serial.println("mDNS responder started");
}


void loop() {
  unsigned long currentMillis = millis();
  limitSwitch.loop();  // MUST call the loop() function first
  
  // put your main code here, to run repeatedly:
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }

  MIDI.read();

  if (limitSwitch.isPressed()) {
    rotationCounter = 0;
    ledcWrite(pwmChannel, 0); // Stop Motor
    Serial.println("The limit switch is touched");
  }

  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
  if (digitalRead(hall_pin) == 0) {
    if (on_state == false) {
      on_state = true;
      if (digitalRead(DIR) == 0) {
        rotationCounter += 1;
      } else {
//        if (rotationCounter != 0) {
          rotationCounter -= 1;
//        } else {
//          Serial.println("Motor moved to position zero. Stopping.");
//          ledcWrite(pwmChannel, 0);
//        }
      }
      Serial.print("rotationCounter:");
      Serial.print(rotationCounter);
      Serial.print(" rotationSet:");
      Serial.println(rotationSet);
      if ((rotationSet >= 0) && (rotationCounter == rotationSet)) {
        ledcWrite(pwmChannel, 0); // Stop Motor
        Serial.print("Motor moved to position and stopped at:");
        Serial.println(rotationSet);
        rotationSet = -1;
      }
    }
  } else {
    on_state = false;
  }


}
