#include <Servo.h>
#include <Wire.h>
#include "IOpins.h"
#include "Constants.h"
#include "fifo.h"

//-------------------------------------------------------------- define global variables --------------------------------------------
int Cmode = CMODE;
unsigned int Volts;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int Leftspeed=0;
int Rightspeed=0;
int Speed;
int Steer;
byte Charged=1;                                               // 0=Flat battery  1=Charged battery
int Leftmode=1;                                               // 0=reverse, 1=brake, 2=forward
int Rightmode=1;                                              // 0=reverse, 1=brake, 2=forward
byte Leftmodechange=0;                                        // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange=0;                                       // Right input must be 1500 before brake or reverse can occur
int LeftPWM;                                                  // PWM value for left  motor speed / brake
int RightPWM;                                                 // PWM value for right motor speed / brake
int data;
int servo[7];

Fifo<byte> i2c_write_bytes;
Fifo<byte> i2c_read_bytes;

//-------------------------------------------------------------- define servos ------------------------------------------------------
Servo Servo0;                                                 // define servos
Servo Servo1;                                                 // define servos
Servo Servo2;                                                 // define servos
Servo Servo3;                                                 // define servos
Servo Servo4;                                                 // define servos
Servo Servo5;                                                 // define servos
Servo Servo6;                                                 // define servos

void setup()
{
  //------------------------------------------------------------ Initialize Servos ----------------------------------------------------
  Servo0.attach(S0);                                          // attach servo to I/O pin
  Servo1.attach(S1);                                          // attach servo to I/O pin
  Servo2.attach(S2);                                          // attach servo to I/O pin
  Servo3.attach(S3);                                          // attach servo to I/O pin
  Servo4.attach(S4);                                          // attach servo to I/O pin
  Servo5.attach(S5);                                          // attach servo to I/O pin
  Servo6.attach(S6);                                          // attach servo to I/O pin

  //------------------------------------------------------------ Set servos to default position ---------------------------------------
  Servo0.writeMicroseconds(DServo0);                          // set servo to default position
  Servo1.writeMicroseconds(DServo1);                          // set servo to default position
  Servo2.writeMicroseconds(DServo2);                          // set servo to default position
  Servo3.writeMicroseconds(DServo3);                          // set servo to default position
  Servo4.writeMicroseconds(DServo4);                          // set servo to default position
  Servo5.writeMicroseconds(DServo5);                          // set servo to default position
  Servo6.writeMicroseconds(DServo6);                          // set servo to default position

  //------------------------------------------------------------ Initialize I/O pins --------------------------------------------------
  pinMode (Charger,OUTPUT);                                   // change Charger pin to output
  digitalWrite (Charger,1);                                   // disable current regulator to charge battery

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

void init_i2c()
{
  // initialize i2c as slave
  Wire.begin(I2C_ADDR);

  // set up i2c callbacks
  Wire.onReceive(receiveI2C);
  Wire.onRequest(sendI2C);
}

void SCmode()
{
  // ------------------------------------------------------------ Code for Serial Communications --------------------------------------
  if (Serial.available() > 1)
  {
    byte A = Serial.read();
    byte B = Serial.read();
    processCommand(A, B);
  }
}

void I2Cmode()
{
  //----------------------------------------------------------- Your code goes here ------------------------------------------------------------
  if (i2c_read_bytes.available() > 1)
  {
    byte A = read_one();
    byte B = read_one();
    processCommand(A, B);
  }
}

// Incoming i2c data callback
void receiveI2C(int byteCount)
{
  int count = byteCount;
  while (count--)
    i2c_read_bytes.enqueue(Wire.read());
}

// Request for i2c data callback
void sendI2C()
{
  if (i2c_write_bytes.available())
    Wire.write(i2c_write_bytes.dequeue()->value);
  else
  {
    Wire.write(-1);
  }
}

void flush()
{
  if (Cmode == CMODE_SERIAL)
  {
    Serial.flush();
  }
  else if (Cmode == CMODE_I2C)
  {
    while (i2c_write_bytes.available())
      i2c_write_bytes.dequeue();

    while (i2c_read_bytes.available())
      i2c_read_bytes.dequeue();
  }
}

void write(byte b)
{
  if (Cmode == CMODE_SERIAL)
    Serial.write(b);
  else if (Cmode == CMODE_I2C)
    i2c_write_bytes.enqueue(b);
}

byte read_one()
{
  if (Cmode == CMODE_SERIAL)
  {
    do
    {
      data = Serial.read();
    } while (data < 0);
    return data;
  }
  else if (Cmode == CMODE_I2C)
  {
    while (!i2c_read_bytes.available());
    data = i2c_read_bytes.dequeue()->value;
    return data;
  }
}

void processCommand(byte A, byte B)
{
  // CH = Change to i2c mode
  // VO = get voltage
  // FL = flush serial buffer
  // AN = report Analog inputs 1-5
  // SV = next 7 integers will be position information for servos 0-6
  // HB = "H" bridge data - next 4 bytes will be:
  //   left  motor mode 0-2
  //   left  motor PWM  0-255
  //   right motor mode 0-2
  //   right motor PWM  0-255

  int command = A*256+B;
  switch (command)
  {
    // This is "CH"; change mode to I2C
    case 17224:
      Cmode = CMODE_I2C;
      init_i2c();

      Serial.write("Mode changed.\n");
      break;

    // This is "VO"; a request for voltage
    case 22095:
      // read the battery voltage (reads 65 for every volt)
      Volts = analogRead(Battery);
      write(Volts / 256);
      write(Volts % 256);
      break;

    // This is "FL"; flush the buffer
    case 17996:
      flush();
      break;

    // This is "AN"; request for the value of the analog pins 1-5
    case 16718:
      for (int i = 1; i < 6; i++)
      {
        // Read the 10-bit analog input
        data = analogRead(i);
        
        // Write each byte one at a time
        write(highByte(data));
        write(lowByte(data));
      }
      break;

    // This is "SV"; set the servo positions
    case 21334:
      // read 14 bytes of data from the user
      for (int i = 0; i < 15; i++)
        servo[i] = read_one();

      // Set the servo positions
      Servo0.writeMicroseconds(servo[0]*256+servo[1]);
      Servo1.writeMicroseconds(servo[2]*256+servo[3]);
      Servo2.writeMicroseconds(servo[4]*256+servo[5]);
      Servo3.writeMicroseconds(servo[6]*256+servo[7]);
      Servo4.writeMicroseconds(servo[8]*256+servo[9]);
      Servo5.writeMicroseconds(servo[10]*256+servo[11]);
      Servo6.writeMicroseconds(servo[12]*256+servo[13]);
      break;

    // This is "HB"; set the PWM data for the motors
    case 18498:
      Leftmode  = read_one();
      LeftPWM   = read_one();
      Rightmode = read_one();
      RightPWM  = read_one();
      break;

    // Invalid
    default:
      flush();
  }
}
