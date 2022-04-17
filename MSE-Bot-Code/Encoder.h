/*
2021 02 04

 encoder implement
  
 */

 #ifndef ENCODER_H
 #define ENCODER_H 1


//#define DEBUG_ENCODER 1
//---------------------------------------------------------------------------

#include "Motion.h";

 

 volatile boolean ENC_btLeftEncoderPulseDataFlag;
 volatile boolean ENC_btRightEncoderPulseDataFlag;
 
 volatile boolean ENC_btLeftMotorRunningFlag;
 volatile boolean ENC_btRightMotorRunningFlag;

 volatile uint16_t ENC_vui16LeftEncoderPulseMissed;
 volatile uint16_t ENC_vui16RightEncoderPulseMissed;


 
 uint16_t ENC_uiAlpha = 8196;

 volatile int32_t ENC_vi32LeftEncoderPulseRawTime;
 volatile int32_t ENC_vi32RightEncoderPulseRawTime;



 
 int32_t ENC_ui32LeftEncoderPulseAveTime;
 int32_t ENC_ui32RightEncoderPulseAveTime;

 uint32_t ENC_ui32LeftEncoderAveTime;
 uint32_t ENC_ui32RightEncoderAveTime;

 volatile int32_t ENC_vi32LeftOdometer;
 volatile int32_t ENC_vi32RightOdometer;

 volatile int32_t ENC_vi32LeftOdometerCompare;
 volatile int32_t ENC_vi32RightOdometerCompare;


 volatile int32_t ENC_vsi32LastTimeLA;
 volatile int32_t ENC_vsi32ThisTimeLA;

void ENC_Calibrate()
{
  
  
  //asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime)); // @ 240mHz clock each tick is ~4nS 



  
}

boolean ENC_ISMotorRunning()
{
  if((ENC_btLeftMotorRunningFlag) && (ENC_btRightMotorRunningFlag))
  {
    return(1);
  }
  else
  {
     return(0);
  }
}

void ENC_SetDistance(int32_t i32LeftDistance, int32_t i32RightDistance)
{
#ifdef DEBUG_ENCODER  
   Serial.print("Set left Odo ");
   Serial.println(i32LeftDistance);
   Serial.print("Set right Odo ");
   Serial.println(i32RightDistance);
   Serial.println("------");
#endif   
   ENC_vi32LeftOdometerCompare = ENC_vi32LeftOdometer + i32LeftDistance;
   ENC_vi32RightOdometerCompare = ENC_vi32RightOdometer + i32RightDistance;
   ENC_btLeftMotorRunningFlag = true;
   ENC_btRightMotorRunningFlag = true;
   ui8LeftWorkingSpeed = cui8StartingSpeed;
   ui8RightWorkingSpeed = cui8StartingSpeed;

}

//Encoder interrupt service routines - entered every change in in encoder pin H-> L and L ->H
//---------------------------------------------------------------------------------------------
void IRAM_ATTR ENC_isrLeftPulse()
{
   volatile static int32_t ENC_vsi32LastTime;
   volatile static int32_t ENC_vsi32ThisTime;
  
   
   // if the last interrupts data wasn't collected, count the miss
  if(ENC_btLeftEncoderPulseDataFlag)
  {
    ENC_vui16LeftEncoderPulseMissed += 1;
    
  }
 
  //how much time elapsed since last interrupt
  
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime )); // @ 240mHz clock each tick is ~4nS 
  ENC_vi32LeftEncoderPulseRawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  ENC_btLeftEncoderPulseDataFlag = true;

  //if Direction pins are high then count up otherwise wheel is going backwards count down
  //odometer reading
  if(digitalRead(ciEncoderLeftDir))
  {
    ENC_vi32LeftOdometer += 1;
  }
  else
  {
    ENC_vi32LeftOdometer -= 1;
  }

  
  if(ENC_btLeftMotorRunningFlag)
  {
    if(ENC_vi32LeftOdometer == ENC_vi32LeftOdometerCompare)
    {
#ifdef DEBUG_ENCODER
      Serial.print("Left Odo ");
      Serial.println(ENC_vi32LeftOdometer);
      Serial.print("Right Odo ");
      Serial.println(ENC_vi32RightOdometer);
      Serial.println("------");
#endif    
      ENC_btLeftMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      digitalWrite(ciMotorLeftA,HIGH);
      digitalWrite(ciMotorLeftB,HIGH);
      digitalWrite(ciMotorRightA,HIGH);
      digitalWrite(ciMotorRightB,HIGH);
      ledcWrite(2,255);
      ledcWrite(1,255);  //stop with braking Left motor 
      ledcWrite(3,255);
      ledcWrite(4,255);  //stop with braking Right motor 
    }
    
  }
  
}



void IRAM_ATTR ENC_isrRightPulse()
{
  volatile static int32_t ENC_vsi32LastTime;
  volatile static int32_t ENC_vsi32ThisTime;

   // if the last interrupts data wasn't collected, count the miss
  if(ENC_btRightEncoderPulseDataFlag)
  {
    ENC_vui16RightEncoderPulseMissed += 1;
  }
  //how much time elapsed since last interrupt
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime)); // @ 240mHz clock each tick is ~4nS 
  ENC_vi32RightEncoderPulseRawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_btRightEncoderPulseDataFlag = true;

 //if Direction pins are high then count up otherwise wheel is going backwards count down
  //odometer reading
  if(digitalRead(ciEncoderRightDir))
  {
    ENC_vi32RightOdometer += 1;
  }
  else
  {
    ENC_vi32RightOdometer -= 1;
  }
  if(ENC_btRightMotorRunningFlag)
  {
    if(ENC_vi32RightOdometer == ENC_vi32RightOdometerCompare)
    {
#ifdef DEBUG_ENCODER      
      Serial.print("Right Odo ");
      Serial.println(ENC_vi32RightOdometer);
      Serial.print("Left Odo ");
      Serial.println(ENC_vi32LeftOdometer);
      Serial.println("------");
#endif    
      ENC_btLeftMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      digitalWrite(ciMotorLeftA,HIGH);
      digitalWrite(ciMotorLeftB,HIGH);
      digitalWrite(ciMotorRightA,HIGH);
      digitalWrite(ciMotorRightB,HIGH);
      ledcWrite(2,255);
      ledcWrite(1,255);  //stop with braking Left motor 
      ledcWrite(3,255);
      ledcWrite(4,255);  //stop with braking Right motor 
    }
    
  }
}


//---------------------------------------------------------------------------------------------

void ENC_Init()
{
   //set pin modes
  pinMode(ciEncoderLeftPulse, INPUT_PULLUP);
  pinMode(ciEncoderLeftDir, INPUT_PULLUP);
  pinMode(ciEncoderRightPulse, INPUT_PULLUP);
  pinMode(ciEncoderRightDir, INPUT_PULLUP);

   // enable GPIO interrupt on change
  attachInterrupt(ciEncoderLeftPulse, ENC_isrLeftPulse, CHANGE);
  attachInterrupt(ciEncoderRightPulse, ENC_isrRightPulse, CHANGE);

  ENC_btLeftMotorRunningFlag = false;
  ENC_btRightMotorRunningFlag = false;


  //check to see if calibration is in eeprom and retreive
  
  
}

void ENC_Disable()
{
   
   // disable GPIO interrupt on change
  detachInterrupt(ciEncoderLeftPulse);
  detachInterrupt(ciEncoderRightPulse);
  
}

int32_t ENC_Averaging()
{
    
  int64_t vi64CalutatedAverageTime;
  
   //yn=yn−1⋅(1−α)+xn⋅α  exponentially weighted moving average IIR Filter 65535 = 1

   //Left Encoder
   
  if(ENC_btLeftEncoderPulseDataFlag)
   {
       
    ENC_btLeftEncoderPulseDataFlag = false;
    
    if (ENC_uiAlpha == 65535 )
    {
      ENC_ui32LeftEncoderPulseAveTime = ENC_vi32LeftEncoderPulseRawTime;
    }
    else
    {
      vi64CalutatedAverageTime = (int64_t)ENC_ui32LeftEncoderPulseAveTime * (65535 - ENC_uiAlpha) + ((int64_t)ENC_vi32LeftEncoderPulseRawTime * ENC_uiAlpha);
      ENC_ui32LeftEncoderPulseAveTime = (int32_t)((vi64CalutatedAverageTime + 32768) / 65536);
   
    }
    if(ENC_ISMotorRunning())
      {
        ENC_ui32LeftEncoderAveTime = (ENC_ui32LeftEncoderPulseAveTime * 3)/1000;
      }
  }

  

  
    //Right Encoder 
  if(ENC_btRightEncoderPulseDataFlag)
   {
    ENC_btRightEncoderPulseDataFlag = false;
    
    if (ENC_uiAlpha == 65535 )
    {
      ENC_ui32RightEncoderPulseAveTime = ENC_vi32RightEncoderPulseRawTime;
    }
    else
    {
      vi64CalutatedAverageTime = (int64_t)ENC_ui32RightEncoderPulseAveTime * (65535 - ENC_uiAlpha) + ((int64_t)ENC_vi32RightEncoderPulseRawTime * ENC_uiAlpha);
      ENC_ui32RightEncoderPulseAveTime = (int32_t)((vi64CalutatedAverageTime + 32768) / 65536);
      
    }
    if(ENC_ISMotorRunning())
      {
        ENC_ui32RightEncoderAveTime = (ENC_ui32RightEncoderPulseAveTime * 3)/1000;
      }
  }

   

  
}


void ENC_ClearLeftOdometer()
{
  ENC_vi32LeftOdometer = 0;
 
}


void ENC_ClearRightOdometer()
{
  ENC_vi32RightOdometer = 0;
}




#endif
