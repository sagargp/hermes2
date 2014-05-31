#include <Servo.h>
#include <Wire.h>

#include "IOpins.h"
#include "Constants.h"

unsigned int  Volts;
unsigned int  LeftAmps;
unsigned int  RightAmps;
unsigned long chargeTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;

int Leftspeed  = 0;
int Rightspeed = 0;
int Speed;
int Steer;

byte Charged         = 1; // 0=Flat battery  1=Charged battery
int Leftmode         = 1; // 0=reverse, 1=brake, 2=forward
int Rightmode        = 1; // 0=reverse, 1=brake, 2=forward
byte Leftmodechange  = 0; // Left input must be 1500 before brake or reverse can occur
byte Rightmodechange = 0; // Right input must be 1500 before brake or reverse can occur

int LeftPWM;  // PWM value for left  motor speed / brake
int RightPWM; // PWM value for right motor speed / brake

// define servos
Servo Servo0;
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;
int servo[7];

void setup()
{
  // Attach all the servos
  Servo0.attach(S0);
  Servo1.attach(S1);
  Servo2.attach(S2);
  Servo3.attach(S3);
  Servo4.attach(S4);
  Servo5.attach(S5);
  Servo6.attach(S6);

  // Set servos to default positions
  Servo0.writeMicroseconds(DSERVO0);
  Servo1.writeMicroseconds(DSERVO1);
  Servo2.writeMicroseconds(DSERVO2);
  Servo3.writeMicroseconds(DSERVO3);
  Servo4.writeMicroseconds(DSERVO4);
  Servo5.writeMicroseconds(DSERVO5);
  Servo6.writeMicroseconds(DSERVO6);

  // Initialize the IO pins
  pinMode(Charger, OUTPUT); // change Charger pin to output
  digitalWrite(Charger, 1); // disable current regulator to charge battery

  // Setup the comms
  Serial.begin(BRATE);
  Serial.flush();
}

void loop()
{
  // Check battery voltage and current draw of motors
  Volts     = analogRead(Battery);
  LeftAmps  = analogRead(LmotorC);
  RightAmps = analogRead(RmotorC);
  
  // Is the left motor current draw exceeding safe limit? Then turn off the motors and record the time
  if (LeftAmps > LEFTMAXAMPS)
  {
    analogWrite(LmotorA, 0);
    analogWrite(LmotorB, 0);
    leftoverload = millis();
  }

  // Is the right motor current draw exceeding safe limit? Then turn off the motors and record the time
  if (RightAmps > RIGHTMAXAMPS)
  {
    analogWrite(RmotorA, 0);
    analogWrite(RmotorB, 0);
    rightoverload = millis();
  }

  // Check the voltage of the battery
  if ((Volts < LOWVOLT) && (Charged == 1))
  {
    // If it's too low, change the status of the battery to "flat".
    // This will shut down the speed controller until the battery is charged again.
    Charged     = 0; // "flat"
    highVolts   = Volts;
    startVolts  = Volts;
    chargeTimer = millis();

    // enable current regulator to charge battery
    digitalWrite(Charger, 0);
  }

  // Charge the battery if the battery is flat and a charger is plugged in (measured voltage is at least 1V higher than last recorded voltage)
  if ((Charged == 0) && (Volts-startVolts > 67))
  {
    // Record the highest voltage at all times (used to detect peak charging)
    if (Volts > highVolts)
    {
      highVolts   = Volts;
      chargeTimer = millis();
    }

    // Battery voltage must be higher than this before peak charging can occur.
    if (Volts > BATVOLT)
    {
      // Detect if we're done charging (battery voltage has levelled out or we've exceeded max charge time)
      if ((highVolts-Volts) > 5 || (millis()-chargeTimer) > CHARGETIMEOUT)
      {
        Charged = 1;
        digitalWrite(Charger, 1);
      }
    } 
  }

  // Battery is not flat and charger is not plugged in. Run the non-battery related firmware
  else
  {
    // If data is available, read a two-byte command and process it
    if (available() > 1)
    {
      int A = read_one();
      int B = read_one();
      processCommand(A, B);
    }

    // Drive the H-bridges
    if (Charged == 1)
    {
      // Only drive the left motor if it's been a while since an overload
      if ((millis()-leftoverload) > OVERLOADTIME)             
      {
        switch (Leftmode)
        {
          // Reverse
          case 0:
            analogWrite(LmotorA, LeftPWM);
            analogWrite(LmotorB, 0);
            break;

          // Brake
          case 1:
            analogWrite(LmotorA, LeftPWM);
            analogWrite(LmotorB, LeftPWM);
            break;

          // Forward
          case 2:
            analogWrite(LmotorA, 0);
            analogWrite(LmotorB, LeftPWM);
            break;
        }
      }

      // Only drive the right motor if it's been a while since an overload
      if ((millis()-rightoverload) > OVERLOADTIME)
      {
        switch (Rightmode)
        {
          // Reverse
          case 0:
            analogWrite(RmotorA, RightPWM);
            analogWrite(RmotorB, 0);
            break;

          // Brake
          case 1:
            analogWrite(RmotorA, RightPWM);
            analogWrite(RmotorB, RightPWM);
            break;

          // Forward
          case 2:
            analogWrite(RmotorA, 0);
            analogWrite(RmotorB, RightPWM);
            break;
        }
      } 
    }
    // Battery is flat, don't drive the motors at all
    else
    {
      analogWrite(LmotorA, 0);
      analogWrite(LmotorB, 0);
      analogWrite(RmotorA, 0);
      analogWrite(RmotorB, 0);
    }
  }
}

int available()
{
  return Serial.available();
}

int read_one()
{
  int data;
  do
  {
    data = Serial.read();
  } while (data < 0);
  return data;
}

void write(byte b)
{
  Serial.write(b);
}

void flush()
{
  Serial.flush();
}

void processCommand(byte A, byte B)
{
  // VO = get voltage
  // FL = flush serial buffer
  // AN = report Analog inputs 1-5
  // SV = next 7 integers will be position information for servos 0-6
  // HB = "H" bridge data - next 4 bytes will be:
  //   left  motor mode 0-2
  //   left  motor PWM  0-255
  //   right motor mode 0-2
  //   right motor PWM  0-255

  int data;
  int command = (A << 8) + B;
  switch (command)
  {
    case COMMAND_VO:
      // read the battery voltage (reads 65 for every volt)
      Volts = analogRead(Battery);
      write(Volts / 256);
      write(Volts % 256);
      break;

    case COMMAND_FL:
      flush();
      break;

    case COMMAND_AN:
      for (int i = 1; i < 6; i++)
      {
        // Read the 10-bit analog input
        data = analogRead(i);
        
        // Write each byte one at a time
        write(highByte(data));
        write(lowByte(data));
      }
      break;

    case COMMAND_SV:
      // read 14 bytes of data from the user
      for (int i = 0; i < 15; i++)
        servo[i] = read_one();

      // Set the servo positions
      Servo0.writeMicroseconds((servo[0]  << 8) + servo[1]);
      Servo1.writeMicroseconds((servo[2]  << 8) + servo[3]);
      Servo2.writeMicroseconds((servo[4]  << 8) + servo[5]);
      Servo3.writeMicroseconds((servo[6]  << 8) + servo[7]);
      Servo4.writeMicroseconds((servo[8]  << 8) + servo[9]);
      Servo5.writeMicroseconds((servo[10] << 8) + servo[11]);
      Servo6.writeMicroseconds((servo[12] << 8) + servo[13]);
      break;

    case COMMAND_HB:
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
