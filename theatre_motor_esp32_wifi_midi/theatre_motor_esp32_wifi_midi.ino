#include <MIDI.h>
#include <AppleMIDI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "esp32-hal-ledc.h"
#include <ezButton.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>

WebServer httpServer(80);
HTTPUpdateServer httpUpdater;

// use Arduino Board: "uPesy ESP32 Wroom DevKit" as device.

// motor name/number
const String motor = "motor1";
// Replace with your network credentials
const char* ssid = "";
const char* password = "";

#define RW_MODE false
#define RO_MODE true
// For non-volatile storage
Preferences prefs;

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
int rotationCounter;
bool rotationSet = false;
int rotationTarget = 0;
bool on_state = false;
int positionFactor = 1;
int rigTopPosition;
int topPosition;
int MotorSpeed = 100;
bool stopMotor = false;
int dir = -1;

APPLEMIDI_CREATE_DEFAULTSESSION_INSTANCE();

//APPLEMIDI_CREATE_INSTANCE(WiFiUDP, MIDI, motor.c_str(), 5004);

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
//  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
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
//      ledcWrite(pwmChannel, 0); // Stop Motor
      Serial.println("Stopping motor");
      stopMotor = true;
      break;
    case 2:
      ledcWrite(pwmChannel, 100);
      digitalWrite(DIR, HIGH);
      Serial.println("Moving motor UP/FORWARD(HIGH)");
      break;
    case 3:
      ledcWrite(pwmChannel, 100);
      digitalWrite(DIR, LOW);
      Serial.println("Moving motor DOWN/BACK(LOW)");
      break;
    case 4:
      Serial.print("Setting Floor(zero) at: 0   | Current position was: ");
      Serial.println(rotationCounter);
      rotationCounter = 0;
     break;
    case 5:
      Serial.print("Setting Stage Top at:");
      Serial.println(rotationCounter);
      topPosition = rotationCounter;
      prefs.putInt("topPosition", topPosition);
     break;
    case 6:
      Serial.print("Setting Rigging Top at:");
      Serial.println(rotationCounter);
      rigTopPosition = rotationCounter;
      prefs.putInt("rigTopPosition", rigTopPosition);
      break;
    case 10:
      Serial.print("Step up:");
      Serial.println(rotationCounter);
      setMotorPosition(rotationCounter + 1);
      break;
    case 11:
      Serial.print("Step down:");
      Serial.println(rotationCounter);
      setMotorPosition(rotationCounter - 1);
      break;
    default:
      // statements
      break;
  }
}
  
void MidiNoteOnCallback(uint8_t channel, uint8_t note, uint8_t velocity)
{
  // velocity in Live only varies between 1-127 (a range of 126)
  int newMotorPosition = (topPosition * (velocity-1)) / 126;

  // Do something with the received MIDI message here
  Serial.print("MIDI: NoteOn: channel:");
  Serial.print(channel);
  Serial.print(" note:");
  Serial.print(note);
  Serial.print(" velocity:");
  Serial.println(velocity);

  if (note==0) {
    MotorSpeed = velocity * 2;
    Serial.print("Setting Motor speed:");
    Serial.println(MotorSpeed);
    if (ledcRead(pwmChannel) > 0) {
      ledcWrite(pwmChannel, MotorSpeed);
    }
    return;
  }
  setMotorPosition(newMotorPosition);
}

void setMotorPosition(int newMotorPosition) {
  
  if (newMotorPosition > rotationCounter) {
    // motor up at MotorSpeed - use "rotationCounter" to manage when it stops
    rotationSet = true;
    rotationTarget = newMotorPosition;
    ledcWrite(pwmChannel, MotorSpeed);
    digitalWrite(DIR, HIGH);
    Serial.print(" Moving UP/Forward(HIGH) to Position:");
    Serial.println(newMotorPosition);
  } else if (newMotorPosition < rotationCounter) {
    // motor DOWN at speed MotorSpeed - use "rotationCounter" to manage when it stops
    rotationSet = true;
    rotationTarget = newMotorPosition;
    ledcWrite(pwmChannel, MotorSpeed);
    digitalWrite(DIR, LOW);
    Serial.print(" Moving DOWN/Back(LOW) to Position:");
    Serial.println(newMotorPosition);
  }
}

void hall_irq_fall() {
  if (digitalRead(DIR) == LOW) { // Down
    if (dir == HIGH) {
      dir = LOW;
      rotationCounter += 1;
    } else {
      dir = LOW;
    }
  }
}

void hall_irq_rise() {
  if (digitalRead(DIR) == LOW) { // Down
    if (dir == HIGH) {
      dir = LOW;
    } else {
      dir = LOW;
      rotationCounter -= 1;
    }
  }
}


void setup() {
  // set the pinmode, this runs once on boot:
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM, pwmChannel);
  pinMode(DIR, OUTPUT);
  pinMode(hall_pin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(hall_pin), hall_irq_fall, FALLING);
//  attachInterrupt(digitalPinToInterrupt(hall_pin), hall_irq_rise, RISING);
  

  // default all to low
  ledcWrite(pwmChannel, 0); // Motor speed 0 to 255
  digitalWrite(DIR, LOW); // LOW for down and HIGH for motor UP

  pinMode(SWITCH_PIN, INPUT_PULLUP); // Only works on non-input-only GPIO PINS
  limitSwitch.setDebounceTime(50);  // set debounce time to 50 milliseconds

  // start the serial port listener
  Serial.begin(115200);
  initWiFi();
  delay(1000);

  prefs.begin("motor", RW_MODE);

  rigTopPosition = prefs.getInt("rigTopPosition", 0);
  topPosition = prefs.getInt("topPosition", 0);
  rotationCounter = prefs.getInt("rotationCounter", 0);
  
  Serial.print("Stored prefs: rigTopPosition: ");
  Serial.print(rigTopPosition);
  Serial.print(", topPosition: ");
  Serial.print(topPosition);
  Serial.print(", rotationCounter: ");
  Serial.println(rotationCounter);


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
  }
  httpServer.on("/", HTTP_GET, []() {
    httpServer.sendHeader("Connection", "close");
    httpServer.send(200, "text/html", "<h1>Motor1</h1>");
  });
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", motor.c_str());

  // Add apple-midi mDNS service so it appears in MIDI browser
  MDNS.addService("_apple-midi", "udp", AppleMIDI.getPort());
  Serial.println("mDNS responder started"); 
   
}


void loop() {
  unsigned long currentMillis = millis();
  limitSwitch.loop();  // MUST call the loop() function first
  httpServer.handleClient();

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
//    rotationCounter = 0;
    ledcWrite(pwmChannel, 0); // Stop Motor
    Serial.println("The limit switch is touched");
    // Store position
    prefs.putInt("rotationCounter", rotationCounter);
  }

  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
  if (digitalRead(hall_pin) == 0) {
    if (on_state == false) {
      on_state = true;
      if (ledcRead(pwmChannel) > 0) {
        if (digitalRead(DIR) == LOW) { // Down
            rotationCounter -= 1;
        } else {
            rotationCounter += 1;
        }
        Serial.print("rotationCounter:");
        Serial.print(rotationCounter);
        Serial.print(" rotationTarget:");
        Serial.println(rotationTarget);
        if ((rotationSet && (rotationCounter == rotationTarget)) || stopMotor) {
          stopMotor = false;
          ledcWrite(pwmChannel, 0); // Stop Motor
          Serial.print("Motor moved to position and stopped at:");
          Serial.println(rotationTarget);
          rotationSet = false;
          // Store position
          prefs.putInt("rotationCounter", rotationCounter);
        }
      }
    }
  } else {
    on_state = false;
  }
}
