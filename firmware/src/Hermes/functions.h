#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

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

Fifo<int> i2c_incoming_buffer;
Fifo<int> i2c_outgoing_buffer;

//-------------------------------------------------------------- define servos ------------------------------------------------------
Servo Servo0;                                                 // define servos
Servo Servo1;                                                 // define servos
Servo Servo2;                                                 // define servos
Servo Servo3;                                                 // define servos
Servo Servo4;                                                 // define servos
Servo Servo5;                                                 // define servos
Servo Servo6;                                                 // define servos

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
  if (Serial.available() > 1)
  {
    int A = Serial.read();
    int B = Serial.read();
    processCommand(A, B);
  }
}

void I2Cmode()
{
  if (i2c_incoming_buffer.available() > 1)
  {
     int A = read_one();
     int B = read_one();
    processCommand(A, B);
  }
}

// Incoming i2c data callback
void receiveI2C(int byteCount)
{
  int count = byteCount;
  while (count--)
    i2c_incoming_buffer.enqueue(Wire.read());
}

// Request for i2c data callback
void sendI2C()
{
  if (i2c_outgoing_buffer.available())
    Wire.write(i2c_outgoing_buffer.dequeue()->value);
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
    while (i2c_outgoing_buffer.available())
      i2c_outgoing_buffer.dequeue();

    while (i2c_incoming_buffer.available())
      i2c_incoming_buffer.dequeue();
  }
}

void write(byte b)
{
  if (Cmode == CMODE_SERIAL)
    Serial.write(b);
  else if (Cmode == CMODE_I2C)
    i2c_outgoing_buffer.enqueue(b);
}

int read_one()
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
    while (!i2c_incoming_buffer.available());
    data = i2c_incoming_buffer.dequeue()->value;
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

#endif
