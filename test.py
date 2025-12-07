import smbus2

bus = smbus2.SMBus(1)
addr = 0x68
WHO_AM_I = 0x75

val = bus.read_byte_data(addr, WHO_AM_I)
print("WHO_AM_I:", hex(val))