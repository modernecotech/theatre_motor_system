#include "BluetoothSerial.h"
#include "esp32-hal-ledc.h"
BluetoothSerial SerialBT;

// motor number
String motor = "M01";

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
  Serial.begin(9600);
  SerialBT.begin(motor);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
    while (SerialBT.available() > 0) {
      String signal = SerialBT.readStringUntil('\n');
      String direction = signal.substring(3,4);
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
    }
}
