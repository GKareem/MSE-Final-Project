
//MSE 2202 
//Western Engineering base code
//2020 05 13 E J Porter


/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage) 
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)                                       Right Motor, Channel A
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                         Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                       Left Encoder, Direction     
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                       Left Encoder, Pulse     Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA  
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK                                                                        
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)                                       Right Motor, Channel B
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0                   
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Pulse
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Encoder, Direction
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;     
//const int ciPB2 = 26;      
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
//const int ciLimitSwitch = 26;
//const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 5;
const int ciMotorRightA = 15;
const int ciMotorRightB = 23;
//const int ciEncoderLeftDir = 18;
//const int ciEncoderLeftPulse = 19;
//const int ciEncoderRightPulse = 14;
//const int ciEncoderRightDir = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciHeartbeatInterval = 500;

const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint16_t CR1_ci8RightTurn = 10000;
const uint16_t CR1_ci8LeftTurn = 10000;

unsigned char CR1_ucMainTimerCaseCore1;


unsigned int CR1_uiMotorRunTime = 1000;


uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLeftEncoderCount = 40000;
unsigned long CR1_ulRightEncoderCount = 40000;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number 
// Argument 3 = Pixel type flags, add together as needed:

//-------------------- Added Variables --------------------

//#include <ESP32Servo.h>
#include <ESP32_Servo.h>
#include <Stepper.h>

// 1 - forward, 2 - left, 3 - right, 4 - reverse

int state = 5;
bool isClimb = false;

//Limit Switches
const int limitSwitchFront = 14;
const int limitSwitchSideA = 26;
const int limitSwitchSideB = 13;

bool limitSwitchFrontState = false;
bool limitSwitchSideAState = false;
bool limitSwitchSideBState = false;

unsigned long currentSwitchTime, preSwitchTime = 0, intervalSwitchTime = 0;
int switchCase = 0;

//Servos
int servoInterval = 5;

int rightServoPin = 18; //Was 12
int leftServoPin = 19;

int rightPos = 180;
int leftPos = 0;

bool rightLimit = false, leftLimit = false;

Servo rightServo;
Servo leftServo;

//Steppers

#define D 25
#define C 17 
#define B 22 
#define A 21 

#define NUMBER_OF_STEPS_PER_REV 31

int stepState = 0;
int stepTimeNow;
int stepTimePrev=0;

void setup() {
   Serial.begin(115200); 
//   Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud
   
   Core_ZEROInit();

   WDT_EnableFastWatchDogCore1();
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[0] = 0;
   WDT_vfFastWDTWarningCore1[1] = 0;
   WDT_vfFastWDTWarningCore1[2] = 0;
   WDT_vfFastWDTWarningCore1[3] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[4] = 0;
   WDT_vfFastWDTWarningCore1[5] = 0;
   WDT_vfFastWDTWarningCore1[6] = 0;
   WDT_vfFastWDTWarningCore1[7] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[8] = 0;
   WDT_vfFastWDTWarningCore1[9] = 0;
   WDT_ResetCore1(); 

   setupMotion();
   pinMode(ciHeartbeatLED, OUTPUT);
   pinMode(ciPB1, INPUT_PULLUP);
//   pinMode(ciLimitSwitch, INPUT_PULLUP);

  //-------------------- Our Code --------------------//
  pinMode(limitSwitchFront, INPUT_PULLUP);
  pinMode(limitSwitchSideA, INPUT_PULLUP);
  pinMode(limitSwitchSideB, INPUT_PULLUP);

  rightServo.attach(rightServoPin, 500, 2400);
  leftServo.attach(leftServoPin, 500, 2400);

  rightLimit = false;
  leftLimit = false;
  isClimb = false;

  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);
  pinMode(C,OUTPUT);
  pinMode(D,OUTPUT);
  
  //-------------------- Our Code --------------------//

   SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
   SmartLEDs.clear();                          // Set all pixel colours to off
   SmartLEDs.show();                           // Send the updated pixel colours to the hardware
}

void loop(){
   //WSVR_BreakPoint(1);

   //average the encoder tick times
   //ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
     CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
    iButtonState = iButtonValue;               // update current button state

     // only toggle the run condition if the new button state is LOW
     if (iButtonState == LOW)
     {
//       ENC_ClearLeftOdometer();
//       ENC_ClearRightOdometer();
       btRun = !btRun;
        Serial.println(btRun);
       // if stopping, reset motor states and stop motors
       if(!btRun)
       {
          ucMotorStateIndex = 0; 
          ucMotorState = 0;
          move(0);
       }
      
     }
   }
 }
 iLastButtonState = iButtonValue; //store button state

// if(!digitalRead(ciLimitSwitch)){
//  btRun = 0; //if limit switch is pressed stop bot
//  ucMotorStateIndex = 0;
//  ucMotorState = 0;
//  move(0);
// }

  //-------------------- Our Code --------------------//
  if(digitalRead(limitSwitchFront)){
    limitSwitchFrontState = false;
  }else{
    limitSwitchFrontState = true;
  }
  
  if(digitalRead(limitSwitchSideA)){
    limitSwitchSideAState = false;
  }else{
    limitSwitchSideAState = true;
  }

  if(digitalRead(limitSwitchSideB)){
    limitSwitchSideBState = false;
  }else{
    limitSwitchSideBState = true;
  }
  //-------------------- Our Code --------------------//
 
 if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
    //Serial.println(iIncomingByte, HEX);     // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
 }
 else
 {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
 }
 
 CR1_ulMainTimerNow = micros();
 if(CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer){
   WDT_ResetCore1(); 
   WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;
   
   CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;
    
   switch(CR1_ucMainTimerCaseCore1)  //full switch run through is 1mS
   {
    //###############################################################################
    case 0:
    {
      
      if(btRun){
//        Serial.print("Front Switch");
//        Serial.println(limitSwitchFrontState);

//        Serial.print("Side A Switch");
//        Serial.println(limitSwitchSideAState);

//        Serial.print("Side B Switch");
//        Serial.println(limitSwitchSideBState);

       //-------------------- Our Code --------------------//
       if (!limitSwitchFrontState && !limitSwitchSideAState && !limitSwitchSideBState && !isClimb){
        state = 1;
        Serial.println("State 1");
       }
       
       if (!limitSwitchFrontState && limitSwitchSideAState && !limitSwitchSideBState && !isClimb){
        state = 2;
//        rightServo.write(180);
//        leftServo.write(0);
        Serial.println("State 2");
        switchCase = 0;
        preSwitchTime = 0;
       }

       if (!limitSwitchFrontState && limitSwitchSideAState && limitSwitchSideBState && !isClimb){
        state = 3;
        Serial.println("State 3");
        switchCase = 0;
        preSwitchTime = 0;
       }

       if (!limitSwitchFrontState && !limitSwitchSideAState && limitSwitchSideBState && !isClimb){
        state = 4;
        Serial.println("State 4");
        switchCase = 0;
        preSwitchTime = 0;
       }

       if (limitSwitchFrontState || isClimb){
        state = 5;
        Serial.println("State 5");
        switchCase = 0;
        preSwitchTime = 0;
        isClimb = true;
       }
       
       if (state == 1){
        currentSwitchTime = millis();
        if (currentSwitchTime - preSwitchTime >= intervalSwitchTime){
          preSwitchTime = currentSwitchTime;
          switch(switchCase){
            case 0:{
              intervalSwitchTime = 1750;
              MoveTo(1, 230, 250);
              switchCase = 1;
              break;
            }
            case 1:{
              intervalSwitchTime = 800;
              MoveTo(2, 200, 200);
              switchCase = 2;
              break;
            }
            case 2:{
              intervalSwitchTime = 1200;
              MoveTo(1, 230, 250);
              switchCase = 0;
              break;
            }
          }
        }
       }else if (state == 2){
        MoveTo(3, 200, 200);
       }else if (state == 3){
        MoveTo(1, 250, 250);
       }else if (state == 4){
        MoveTo(2, 200, 200);
       }else if (state == 5){
        move(0);
        CR1_ulMotorTimerNow = millis();
        stepTimeNow = millis();
        runSteppers();
        runServos();
       }
       //-------------------- Our Code --------------------//
       
//       CR1_ulMotorTimerNow = millis();
//       if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_uiMotorRunTime){



      
//         switch(ucMotorStateIndex)
//         {
//          case 0:
//          {
//            
//            CR1_uiMotorRunTime = 1000;
//            ucMotorStateIndex = 1;
//            ucMotorState = 0;
//            move(0);
//            break;
//          }
//           case 1:
//          {
//            CR1_uiMotorRunTime = 5000;
//            ENC_SetDistance(CR1_ulLeftEncoderCount, CR1_ulRightEncoderCount);
//            Serial.print("Wheel speeds ");
//            Serial.println(CR1_ui8WheelSpeed);
//            Serial.print("------------");
//            ucMotorState = 1;   //forward
//            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
//            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
//            ucMotorStateIndex = 2;
//                     
//            break;
//          }
//           case 2:
//          {
//            CR1_uiMotorRunTime = 1000;
//            ucMotorStateIndex = 3;
//            ucMotorState = 0;
//            move(0);
//            break;
//          }
//          case 3:
//          {
//            CR1_uiMotorRunTime = 2000;
//            ENC_SetDistance(-(CR1_ci8LeftTurn), CR1_ci8LeftTurn);
//            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
//            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
//            ucMotorStateIndex = 4;
//            ucMotorState = 2;  //left
//           
//            break;
//          }
//           case 4:
//          {
//            CR1_uiMotorRunTime = 1000;
//            ucMotorStateIndex = 5;
//            ucMotorState = 0;
//            move(0);
//            break;
//          }
//         case 5:
//          {
//            CR1_uiMotorRunTime = 2000;
//            ENC_SetDistance(CR1_ci8RightTurn,-(CR1_ci8RightTurn));
//            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
//            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
//            ucMotorStateIndex =  6;
//            ucMotorState = 3;  //right
//            
//            break;
//          }
//          case 6:
//          {
//            CR1_uiMotorRunTime =  1000;
//            ucMotorStateIndex = 7;
//            ucMotorState = 0;
//            move(0);
//            break;
//          }
//           case 7:
//          {
//            CR1_uiMotorRunTime = 5000;
//            ucMotorStateIndex = 8;
//            ucMotorState = 4;  //reverse
//            ENC_SetDistance(-(CR1_ulLeftEncoderCount), -(CR1_ulRightEncoderCount));
//            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
//            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
//            
//            break;
//          }
//          case 8:
//          {
//            CR1_uiMotorRunTime = 1000;
//            ucMotorStateIndex = 9;
//            ucMotorState = 0;
//            move(0);
//            break;
//          }
//          case 9:
//          {
//            CR1_uiMotorRunTime = 2000;
//            ENC_SetDistance(CR1_ci8RightTurn,-(CR1_ci8RightTurn));
//            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
//            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
//            ucMotorStateIndex = 10;
//            ucMotorState = 3;  //right
//            
//            break;
//          }
//          case 10:
//          {
//            CR1_uiMotorRunTime = 1000;
//            ucMotorStateIndex = 11;
//            ucMotorState = 0;
//            move(0);
//            break;
//          }
//           case 11:
//          {
//            CR1_uiMotorRunTime = 2000;
//            ENC_SetDistance(-(CR1_ci8LeftTurn), CR1_ci8LeftTurn);
//            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
//            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
//            ucMotorStateIndex = 0;
//            ucMotorState = 2;  //left
//            
//            break;
//          }
//         }
//        }
      }
      CR1_ucMainTimerCaseCore1 = 1;
      
      break;
    }
    //###############################################################################
    case 1: 
    {
      //read pot 1 for motor speeds 
      //CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255); //adjust to range that will produce motion
      CR1_ui8WheelSpeed = 250;
      CR1_ucMainTimerCaseCore1 = 2;
      break;
    }
    //###############################################################################
    case 2: 
    {
     // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS 
     
   //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS 
     
      CR1_ucMainTimerCaseCore1 = 3;
      break;
    }
    //###############################################################################
    case 3: 
    {
      
      //MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed,CR1_ui8RightWheelSpeed);
   
      CR1_ucMainTimerCaseCore1 = 4;
      break;
    }
    //###############################################################################
    case 4:   
    {
    
      CR1_ucMainTimerCaseCore1 = 5;
      break;
    }
    //###############################################################################
    case 5: 
    {
      
     
      CR1_ucMainTimerCaseCore1 = 6;
      break;
    }
    //###############################################################################
    case 6:
    {
  
    
      CR1_ucMainTimerCaseCore1 = 7;
      break;
    }
    //###############################################################################
    case 7: 
    {
       if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
         SmartLEDs.setPixelColor(0,25,0,0);         // make LED1 green with 10% intensity
       }
       else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
         SmartLEDs.setPixelColor(0,25,0,25);        // make LED1 purple with 10% intensity
       }
       else {                                       // otherwise
         SmartLEDs.setPixelColor(0,0,25,0);         // make LED1 red with 10% intensity
       }
       SmartLEDs.show();                            // send updated colour to LEDs
          
      CR1_ucMainTimerCaseCore1 = 8;
      break;
    }
    //###############################################################################
    case 8: 
    {
    
      CR1_ucMainTimerCaseCore1 = 9;
      break;
    }
    //###############################################################################
    case 9: 
    {
  
      CR1_ucMainTimerCaseCore1 = 0;
      break;
    }

  }
 }

 // Heartbeat LED
 CR1_ulHeartbeatTimerNow = millis();
 if(CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
 {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
   // Serial.println((vui32test2 - vui32test1)* 3 );
 }
}

void runServos(){
  if (!rightLimit && !leftLimit){
    for (rightPos = 180, leftPos = 0; rightPos >= 80, leftPos <= 100; rightPos -= 1, leftPos += 1){
      rightServo.write(rightPos);
      if (rightPos <= 80) rightLimit = true;
            
      leftServo.write(leftPos);
      if (leftPos >= 100) leftLimit = true;
            
      CR1_uiMotorRunTime = 10;
      CR1_ulMotorTimerPrevious = millis();
      while (millis() - CR1_ulMotorTimerPrevious < CR1_uiMotorRunTime);
    }
  }
}

void runSteppers(){
  if(stepTimeNow - stepTimePrev >= 1){
    stepTimePrev = stepTimeNow;
    if(stepState == 0){
      write(1, 0, 0, 0);
      stepState = 1;
    }else if (stepState == 1){
      write(1, 1, 0, 0);
      stepState = 2;
    }else if (stepState == 2){
      write(0, 1, 0, 0);
      stepState=3;
    }else if (stepState == 3){
      write(0, 1, 1, 0);
      stepState=4;
    }else if (stepState == 4){
      write(0, 0, 1, 0);
      stepState=5;
    }else if (stepState == 5){
      write(0, 0, 1, 1);
      stepState = 6;
    }else if (stepState == 6){
      write(0, 0, 0, 1);
      stepState = 7;
    }else if (stepState == 7){
      write(1, 0, 0, 1);
      stepState = 0;
    }
  }
}

void write(int a,int b,int c,int d){
  digitalWrite(A, a);
  digitalWrite(B, b);
  digitalWrite(C, c);
  digitalWrite(D, d);
}
