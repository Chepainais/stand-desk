#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Encoder.h>

// DISPLAY
#include <Adafruit_SSD1306.h>
#include <display.cpp>
// MOTORS
#include "Motor.h"
#include "MotorControl.h"
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ENCODERS
ESP32Encoder encoder;
ESP32Encoder encoder2;
MotorControl mc;
// MOTOR CONTROL CONSTANTS
#define MOTOR_DIRECTION_UP 0;
#define MOTOR_DIRECTION_DOWN 1;
#define MOTOR_RPM_CM_RATIO 10; // guess for now
#define PWM_MIN 30;
#define PWM_MAX 100;
#define MOTOR_SOFT_START_RATION 10;
// EEPROM
int EEPROM_SIZE = 12;
int EEPROMEnc1Pos = 0;
int EEPROMEnc2Pos = 0;

// GPIOs
int enc1GPIOG = 18;
int enc1GPIOB = 19;
int enc2GPIOG = 16;
int enc2GPIOB = 17;
int dcPwm1GPIO = 25;
int dcDir1GPIO = 14;
int dcPwm2GPIO = 26;
int dcDir2GPIO = 12;
int btnUpGPIO = 34;
int btnDownGPIO = 35;

// APP variables, constants
bool RESET = false;
int lastEnc1Pos = 0;
int lastEnc2Pos = 0;
int currentPosition = 0;
int currentPositiontarget = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("Start set up");
      // INIT input buttons
        // UP | primary
        pinMode(btnUpGPIO, INPUT);
        pinMode(btnDownGPIO, INPUT);
        // DOWN | primary
        // SAVE position | optional
        // GOTO position | optional
        // RESET MIN/MAX position | primary
        // PWM change | optional
      pinMode(dcDir1GPIO, OUTPUT);
      pinMode(dcDir2GPIO, OUTPUT);
      pinMode(dcPwm1GPIO, OUTPUT);
      pinMode(dcPwm2GPIO, OUTPUT);
      pinMode(dcDir1GPIO, OUTPUT);
      pinMode(dcDir2GPIO, OUTPUT);
      // TEST AREA

      // @TODO INIT DC MOTOR OUTPUT PINS

      // INIT EEPROM
        // READ LAST SAVED EEPROM VALUES
    EEPROM.begin(EEPROM_SIZE);
    // lastEnc1Pos = EEPROM.readInt(EEPROMEnc1Pos);
    lastEnc1Pos = 33;
    // lastEnc2Pos = EEPROM.readInt(EEPROMEnc2Pos);
    lastEnc2Pos = 33;
    Serial.print("Enc saved pos 1: ");
    Serial.println(lastEnc1Pos);
    Serial.print("Enc saved pos 2: ");
    Serial.println(lastEnc2Pos);
      // INIT encoders, set start positions
      //ESP32Encoder::useInternalWeakPullResistors=DOWN;
      // Enable the weak pull up resistors
      ESP32Encoder::useInternalWeakPullResistors=UP;

      // use pin 19 and 18 for the first encoder
      encoder.attachHalfQuad(enc1GPIOB, enc1GPIOG);
      // use pin 17 and 16 for the second encoder
      encoder2.attachHalfQuad(enc2GPIOB, enc2GPIOG);
        
      // set starting count value after attaching
      encoder.setCount(lastEnc1Pos);
      encoder2.setCount(lastEnc2Pos);
        // nrpk = pwm channel for ledc
      Motor m1 = Motor(encoder,dcPwm1GPIO,dcDir1GPIO, 0);
      // Motor m2 = Motor(encoder2,dcPwm2GPIO,dcDir2GPIO, 1);
      mc.addMotor(m1);
      // mc.addMotor(m2);
      Serial.println("All set up");
      // INIT display
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

}

void loop() {
    // check encoder values, store at EEPROM if changed
    int enc1Position = encoder.getCount();
    // int enc2Position = encoder2.getCount();
    // currentPosition = max(enc1Position, enc2Position);
    currentPosition = enc1Position;

    // Check up/down buttons
    int btnUpState = digitalRead(btnUpGPIO);
    Serial.print("Btn UP state: ");
    Serial.println(btnUpState);
    int btnDownState = digitalRead(btnDownGPIO);
    Serial.print("Btn DOWN state: ");
    Serial.println(btnDownState);
    // If both buttons is pressed
    // Reset position
    if(btnUpState && btnDownState){
      currentPosition = 0;
      currentPositiontarget = 0;
       return;
    }
    if(btnUpState){
      // @TODO move step to constant
      currentPositiontarget = currentPosition + 200;
    }
    if(btnDownState){
      currentPositiontarget = currentPosition - 200;
    }
    Serial.print("Cur pos: ");
    Serial.print(currentPosition);
    Serial.print(" target: ");
    Serial.println(currentPositiontarget);
    // SAVE LAST POSITION TO EEPROM, IF DIFFERENT
    // if(enc1Position != lastEnc1Pos || enc2Position != lastEnc2Pos){
    if(enc1Position != lastEnc1Pos){
      EEPROM.writeInt(EEPROMEnc1Pos, enc1Position);
      // EEPROM.writeInt(EEPROMEnc2Pos, enc2Position);
      Serial.println("EEPROM write");
      EEPROM.commit();
    }
    lastEnc1Pos = enc1Position;
    // lastEnc2Pos = enc2Position;
    // MOTOR control section
      // if position target not reached - try 2 reach

    // Dont move if small diff
    if(abs(currentPositiontarget - currentPosition) > 30){
        Serial.print("move to height");
        Serial.println(currentPositiontarget);
      mc.moveToHeight(currentPositiontarget);
    }
    // display new height
    delay(1000);
    //

}