import smbus

bus = smbus.SMBus(1)
address = 0X04

bus.write_byte(address, 'V')
bus.write_byte(address, 'O')

response_0 = bus.read_byte(address)
response_1 = bus.read_byte(address)

print ord(response_0) * 256 + ord(response_1)
