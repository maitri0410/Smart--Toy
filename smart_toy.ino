//******************libraries********************//
#include <Talkie.h>       //audio library
#include <TalkieUtils.h>
#include <Vocab_US_Large.h>
#include <Vocab_Special.h>

#include <tcs3200.h>     //color sensor
#include <NewPing.h>     //ultrasonic sensor
#include <IRremote.h>    //IR receiver

#include <Wire.h>   //I2C
#include <RTClib.h> // RTC
#include <toneAC.h> // Library for generating audio tones

//**************pin assignment*************//

//ultrasonic sensor pins
#define trig_pin A3 //analog output 
#define echo_pin A2 //analog input

// L298N motor driver control pins
const int LeftMotorForward   = 2;
const int LeftMotorBackward  = 3;
const int RightMotorForward  = 4;
const int RightMotorBackward = 5;

//color sensor TCS3200
const int tcs_S2             = 6;
const int tcs_S3             = 7;
const int tcs_sensorOut      = 8;

//IR receiver
const int ir_signal          = 9;

//audio module
const int audio_out          = 10;

//*************** define variables and constants***************

const int waterInterval = 3600; // 1 hour in seconds ( for alarm)

//ultrasonic sensor pins
#define maximum_distance 200
int distance = 100;

// L298N motor driver control pins
boolean goesForward = false;

//color sensor TCS3200
// Calibration Values
// *Get these from Calibration
int redMin = 0; // Red minimum value
int redMax = 0; // Red maximum value
int greenMin = 0; // Green minimum value
int greenMax = 0; // Green maximum value
int blueMin = 0; // Blue minimum value
int blueMax = 0; // Blue maximum value

// *Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

//*Variables for final Color values
int redValue;
int greenValue;
int blueValue;

//IR receiver
unsigned long key_value = 0;
#define turnLeft_freq 0x000001 //get these from calibration
#define turnRight_freq 0x000002 //get these from calibration
#define turnForward_freq 0x000003 //get these from calibration
#define detectColor_freq 0x000004 //get these from calibration


//audio module
const uint8_t spRED[] PROGMEM   = {0x64,0x94,0x76,0xD2,0x24,0xD2,0xA8,0x45,0x97,0x94,0x72,0x95,0xAB,0x26,0x51,0x9B,0x32,0xCC,0x89,0x9A,0x64,0x0B,0xCE,0x34,0x67,0x61,0xF4,0x4D,0xA4,0x43,0x9C,0x8F,0x3E,0x0F,0xB6,0xB0,0x70,0xBF,0xBA,0x12,0x53,0xC2,0xAD,0xFB,0xEA,0x9A,0x19,0xD1,0xB4,0x2E,0xAB,0xEF,0x76,0x59,0xC2,0x76,0xAF,0xA1,0xAB,0x61,0x29,0xDF,0xB5,0x86,0xA1,0x86,0x35,0xFC,0xD3,0xEA,0x27,0x1F,0x96,0xB0,0x4F,0xA2,0x33,0xAC,0x7D,0x4C,0x43,0x89,0x5E,0xCA,0x8C,0x11,0x0F,0x5D,0xBA,0xC9,0x44,0x25,0x2C,0x49,0x69,0x9A,0x48,0x16,0xCF,0x47,0xAD,0xAD,0xD6,0x45,0xB5,0x66,0x99,0xA6,0x26,0x67,0x86,0x9C,0xAC,0xDA,0x96,0x92,0x99,0x2C,0xF2,0xFF,0x03};
const uint8_t spGREEN[] PROGMEM = {0x64,0xD5,0xA2,0x22,0x23,0xAC,0xB0,0x4D,0xF1,0xCA,0x2C,0x55,0x1A,0xF6,0x6C,0x3F,0x24,0xC4,0x72,0x19,0xB2,0xCA,0xA0,0x62,0x67,0xAD,0x8B,0x49,0xCD,0x53,0xDC,0x8D,0x3A,0x55,0x0E,0x4D,0xB5,0x3F,0xAA,0xD2,0x38,0x5C,0xBD,0xFD,0xAA,0x5A,0x51,0x76,0xB7,0x2D,0xA3,0xEE,0xC5,0xD1,0xD2,0x6F,0xAD,0x66,0x78,0x43,0xDB,0x28,0x35,0xDA,0x61,0x15,0xED,0x22,0x4C,0x6B,0x87,0x15,0x8A,0x73,0xB3,0xAD,0xEB,0x52,0xB9,0x4E,0xAD,0xB6,0xAE,0x29,0xB2,0x09,0x0B,0x5B,0xDA,0xA6,0xB1,0xCA,0xDC,0x69,0x69,0xAB,0xC2,0x0A,0x73,0xD5,0xA5,0x6D,0x9A,0xD5,0x3D,0x12,0x95,0xB6,0x7A,0x15,0xB7,0x8A,0x58,0xDA,0xE2,0x54,0x32,0x42,0x62,0x69,0x8A,0x13,0x35,0xCD,0x48,0xFF,0x0F};
const uint8_t spBLUE[] PROGMEM  = {0x2D,0x0B,0x81,0xC7,0x94,0xD5,0x8F,0x2A,0x27,0x1C,0x27,0x71,0x3F,0xAB,0x55,0x59,0x97,0xA1,0x22,0x79,0x77,0xB5,0xAB,0x59,0xCF,0xB6,0x77,0xA1,0x17,0xB6,0x4B,0x03,0xB3,0xFB,0x9E,0x0D,0x2C,0x6C,0x4A,0x67,0x92,0x07,0x00,0x00};

//*******************initializations ***********************//

//RTC for alarm
RTC_DS3231 rtc;

//ultrasonic sensor
NewPing sonar(trig_pin, echo_pin, maximum_distance);  //sensor function

//IR receiver
IRrecv irrecv(ir_signal);
decode_results results;

//audio module
Talkie voice;

//**************setup********************************//
  void setup()
  {
    //RTC for alarm
    if (!rtc.begin()) 
    {
    Serial.println("Couldn't find RTC");
    while (1);
    }
  
  //ultraonic sensor
  distance = readPing();

  //motor driver
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  //color sensor
  // Set S2, S3 as outputs
	pinMode(tcs_S2, OUTPUT);
	pinMode(tcs_S3, OUTPUT);

	// Set Sensor output as input
	pinMode(tcs_sensorOut, INPUT);

  //IR receiver
  irrecv.enableIRIn();
  irrecv.blink13(true);
  Serial.begin(9600);
}

void loop() 
{
  DateTime now = rtc.now();
  
  int secondsSinceLastWater = now.hour() * 3600 + now.minute() * 60 + now.second();
  
  if (secondsSinceLastWater >= waterInterval) {
    playWaterSound();
    delay(5000); // Wait for 5 seconds (adjust as needed)
  }
  if (irrecv.decode(&results))
  {
        if (results.value == 0XFFFFFFFF)
          results.value = key_value;

        switch(results.value)
        {
          case turnLeft_freq:
          turnLeft();
          break;

          case turnRight_freq:
          turnRight();
          break;

          case turnForward_freq:
          moveForward();
          break;

          case detectColor_freq:
          {
            redPW = getRedPW();                               // Read Red value
            redValue = map(redPW, redMin,redMax,255,0);       // Map to value from 0-255
            delay(200);                                       // Delay to stabilize sensor

            greenPW = getGreenPW();                            // Read Green value
            greenValue = map(greenPW, greenMin,greenMax,255,0);// Map to value from 0-255
            delay(200);                                        // Delay to stabilize sensor

            bluePW = getBluePW();                              // Read Blue value
            blueValue = map(bluePW, blueMin,blueMax,255,0);    // Map to value from 0-255
            delay(200);                                        // Delay to stabilize sensor

            //decide final color
            if(redValue>greenValue){
              if(redValue>blueValue)
                speakRed();                               // red detected
                else 
                speakBlue();                              // blue detected
             }
            else if(greenValue>blueValue)
               speakGreen();                              //green detected
               else
               speakBlue();                               //blue detected
          }
        } // end case
  } //end if
  if (distance <= 20)
          {
            moveStop();
            delay(300);
          }
          distance = readPing();
} 


//*************required functions***********************//

//  //  //  //  //  AUDIO MODULE   // //  //  //  //  //
void speakRed() {
  voice.say(spRED);
  while (voice.isTalking())
    ;
  
  Serial.println("Red");
}
void speakGreen() {
  voice.say(spGREEN);
  while (voice.isTalking())
    ;
  
  Serial.println("Green");
}
void speakBlue() {
  voice.say(spBLUE);
  while (voice.isTalking())
    ;
  
  Serial.println("Blue");
}

//  //  //  //  ULTRASONIC SENSOR //  //  //  //  //
int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  return cm;
}

//  //  //  // MOTOR DRIVER //  //  //  //  //
void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(){

  if(!goesForward){

    goesForward=true;
    
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
  
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW); 
  }
}
void turnRight(){

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  delay(500);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
 
}

void turnLeft(){

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(500);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

//  //  //  // COLOR SENSOR //  //  //  //  //
// Function to read Red Pulse Widths
int getRedPW() {
	// Set sensor to read Red only
	digitalWrite(tcs_S2,LOW);
	digitalWrite(tcs_S3,LOW);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(tcs_sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
	// Set sensor to read Green only
	digitalWrite(tcs_S2,HIGH);
	digitalWrite(tcs_S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(tcs_sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
	// Set sensor to read Blue only
	digitalWrite(tcs_S2,LOW);
	digitalWrite(tcs_S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(tcs_sensorOut, LOW);
	// Return the value
	return PW;
}

//function to play alarm
void playWaterSound() {
  toneAC(audio_out, 1000, 1000); // Play a 1kHz tone for 1 second
}
