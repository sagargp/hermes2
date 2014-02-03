import smbus

bus     = smbus.SMBus(1)
address = 0x40

bus.write_byte(address, ord('A'))
bus.write_byte(address, ord('D'))
bus.write_byte(address, 2)
bus.write_byte(address, 3)

response = bus.read_byte(address)
print response
