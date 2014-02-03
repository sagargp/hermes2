#include <Wire.h>
#include "fifo.h"

#define I2C_ADDR 0x40

Fifo<byte> i2c_incoming_buffer;
Fifo<byte> i2c_outgoing_buffer;

byte value;

void setup()
{
  Serial.begin(9600);
  Serial.flush();

  // initialize I2C as slave
  Wire.begin(I2C_ADDR);

  // set up I2C callbacks
  Wire.onReceive(receiveI2C);
  Wire.onRequest(sendI2C);

  value = 0;
}

void loop()
{
  if (i2c_incoming_buffer.available())
  {
    byte A = get_byte();
    byte B = get_byte();

    char b[3];
    itoa(A, b, 10);

    Serial.write("{");
    Serial.write(b);
    Serial.write(", ");
    itoa(B, b, 10);
    Serial.write(b);
    Serial.write("}\n");

    if (A == 'A' && B == 'D')
    {
      Serial.write("ad\n");
      value++;
      i2c_outgoing_buffer.enqueue(value);
    }
  }
}

byte get_byte()
{
  while (!i2c_incoming_buffer.available());
  return i2c_incoming_buffer.dequeue();
}

// Incoming i2c data callback
void receiveI2C(int byteCount)
{
  //while (byteCount--)
  i2c_incoming_buffer.enqueue(Wire.read());
}

// Request for i2c data callback
void sendI2C()
{
  if (i2c_outgoing_buffer.available())
    Wire.write(i2c_outgoing_buffer.dequeue());
  else
    Wire.write(-1);
}
