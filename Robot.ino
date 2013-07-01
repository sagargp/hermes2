#include "circbuf.h"
#include <string>

#define WATCHDOG_THRESHOLD 3000

// left/right
int In1 = 6;
int In2 = 9;

// forward/back
int In3 = 10;
int In4 = 11;

byte line[5]; // cmd ID, x_sign, x_byte, y_sign, y_byte

unsigned long watchdog;
CircularBuffer<8> buffer;

bool debug = false;

enum packetid
{
  ID_ERROR  = 1,
  ID_MOTOR  = 98,
  ID_START  = 255,
  ID_END    = 254,
  ID_CONFIG = 99
};

void setMotors(byte x_sign, byte x, byte y_sign, byte y)
{
  // x - rotational component
  // y - linear component 

  if (x_sign == 0) // positive
  {
    analogWrite(In1, x);
    analogWrite(In2, 0);
  }
  else
  {
    analogWrite(In1, 0);
    analogWrite(In2, x);
  }

  if (y_sign == 0) // positive
  {
    analogWrite(In3, y);
    analogWrite(In4, 0);
  }
  else
  {
    analogWrite(In3, 0);
    analogWrite(In4, y);
  }
}

bool checksumOK()
{
  byte checksum = 0;
  for(int i=0; i<buffer.capacity()-1; ++i)
    checksum ^= buffer.peek(i);
  return checksum == buffer.peek(buffer.capacity()-1);
}

void readLine(byte *line, int size)
{
  buffer.read(); // start bit (should be 255)
  for (int i = 0; i < size; i++)
    line[i] = buffer.read();
  buffer.read(); // end bit (should be 254)
}

void config(byte toggle)
{
  if (toggle)
    debug = true;
  else
    debug = false;
}

void setup()
{
  Serial.begin(9600);
  watchdog = millis();
}

void loop()
{
  if (Serial.available() > 0)
    buffer.write(Serial.read());

  if (buffer.isFull() && buffer.peek(0) == 255 && checksumOK())
  {
    if (debug)
    {
      Serial.write("ret:");
      for (int i = 0; i < 8; i++)
        Serial.write(buffer.peek(i));
      Serial.write("\n");
    }

    readLine(line, sizeof(line));
    switch (line[0])
    {
      case ID_MOTOR:
        setMotors(line[1], line[2], line[3], line[4]);
        break;

      case ID_CONFIG:
        config(line[1]);
        break;

      default:
        setMotors(0, 0, 0, 0);
        break;
    }

    // whatever 
    while (Serial.available() > 0)
      Serial.read();
    watchdog = millis();
  }

  if (millis() - watchdog > WATCHDOG_THRESHOLD)
  {
    if (debug)
      Serial.write("timeout\n");
    setMotors(0, 0, 0, 0);
  }
  Serial.flush();
}
