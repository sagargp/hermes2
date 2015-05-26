#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "IOpins.h"
#include "Constants.h"
#include "FifoLib.h"
#include "functions.h"

void setup()
{
  Servo0.attach(S0);
  Servo1.attach(S1);
  Servo2.attach(S2);
  Servo3.attach(S3);
  Servo4.attach(S4);
  Servo5.attach(S5);
  Servo6.attach(S6);

  // Set servos to default position
  Servo0.writeMicroseconds(DServo0);
  Servo1.writeMicroseconds(DServo1);
  Servo2.writeMicroseconds(DServo2);
  Servo3.writeMicroseconds(DServo3);
  Servo4.writeMicroseconds(DServo4);
  Servo5.writeMicroseconds(DServo5);
  Servo6.writeMicroseconds(DServo6);

  // Initialize I/O pins
  pinMode (Charger,OUTPUT);                                   // change Charger pin to output
  digitalWrite(Charger, 1);                                   // disable current regulator to charge battery

  if (Cmode == CMODE_SERIAL) 
  {
    Serial.begin(Brate);                                      // enable serial communications if Cmode=1
    Serial.flush();                                           // flush buffer
  } 
  else if (Cmode == CMODE_I2C)
  {
    init_i2c();
  }
}

void loop()
{
  //------------------------------------------------------------ Check battery voltage and current draw of motors ---------------------
  Volts     = analogRead(Battery);                            // read the battery voltage
  LeftAmps  = analogRead(LmotorC);                            // read left motor current draw
  RightAmps = analogRead(RmotorC);                            // read right motor current draw

  if (LeftAmps>Leftmaxamps)                                   // is motor current draw exceeding safe limit
  {
    analogWrite(LmotorA,0);                                  // turn off motors
    analogWrite(LmotorB,0);                                  // turn off motors
    leftoverload=millis();                                    // record time of overload
  }

  if (RightAmps>Rightmaxamps)                                 // is motor current draw exceeding safe limit
  {
    analogWrite (RmotorA,0);                                  // turn off motors
    analogWrite (RmotorB,0);                                  // turn off motors
    rightoverload=millis();                                   // record time of overload
  }

  // isUnderVoltage()
  if ((Volts<lowvolt) && (Charged==1))                        // check condition of the battery
  {                                                           // change battery status from charged to flat
    //---------------------------------------------------------- FLAT BATTERY speed controller shuts down until battery is recharged ----
    //---------------------------------------------------------- This is a safety feature to prevent malfunction at low voltages!! ------
    Charged=0;                                                // battery is flat
    highVolts=Volts;                                          // record the voltage
    startVolts=Volts;
    chargeTimer=millis();                                     // record the time

    digitalWrite (Charger,0);                                 // enable current regulator to charge battery
  }

  //------------------------------------------------------------ CHARGE BATTERY -------------------------------------------------------
  // isPluggedIn() && !isFullyCharged()
  if ((Charged==0) && (Volts-startVolts>67))                  // if battery is flat and charger has been connected (voltage has increased by at least 1V)
  {
    if (Volts>highVolts)                                      // has battery voltage increased?
    {
      highVolts=Volts;                                        // record the highest voltage. Used to detect peak charging.
      chargeTimer=millis();                                   // when voltage increases record the time
    }

    if (Volts>batvolt)                                        // battery voltage must be higher than this before peak charging can occur.
    {
      if ((highVolts-Volts)>5 || (millis()-chargeTimer)>chargetimeout) // has voltage begun to drop or levelled out?
      {
        Charged=1;                                            // battery voltage has peaked
        digitalWrite (Charger,1);                             // turn off current regulator
      }
    } 
  }

  else
  {
    //----------------------------------------------------------- GOOD BATTERY speed controller opperates normally ----------------------
    switch(Cmode)
    {
      case CMODE_SERIAL:
        SCmode();
        break;

      case CMODE_I2C:                                                   // I2C mode via A4(SDA) and A5(SCL)
        I2Cmode();
        break;
    }

    // --------------------------------------------------------- Code to drive dual "H" bridges --------------------------------------
    if (Charged==1)                                           // Only power motors if battery voltage is good
    {
      if ((millis()-leftoverload)>overloadtime)             
      {
        switch (Leftmode)                                     // if left motor has not overloaded recently
        {
          case 2:                                               // left motor forward
            analogWrite(LmotorA,0);
            analogWrite(LmotorB,LeftPWM);
            break;

          case 1:                                               // left motor brake
            analogWrite(LmotorA,LeftPWM);
            analogWrite(LmotorB,LeftPWM);
            break;

          case 0:                                               // left motor reverse
            analogWrite(LmotorA,LeftPWM);
            analogWrite(LmotorB,0);
            break;
        }
      }

      if ((millis()-rightoverload)>overloadtime)
      {
        switch (Rightmode)                                    // if right motor has not overloaded recently
        {
          case 2:                                               // right motor forward
            analogWrite(RmotorA,0);
            analogWrite(RmotorB,RightPWM);
            break;

          case 1:                                               // right motor brake
            analogWrite(RmotorA,RightPWM);
            analogWrite(RmotorB,RightPWM);
            break;

          case 0:                                               // right motor reverse
            analogWrite(RmotorA,RightPWM);
            analogWrite(RmotorB,0);
            break;
        }
      } 
    }
    else                                                      // Battery is flat
    {
      analogWrite (LmotorA,0);                                // turn off motors
      analogWrite (LmotorB,0);                                // turn off motors
      analogWrite (RmotorA,0);                                // turn off motors
      analogWrite (RmotorB,0);                                // turn off motors
    }
  }
}
