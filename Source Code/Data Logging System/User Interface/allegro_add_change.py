#Author: Nicholas Altwasser
#SSID: 200389010
#
#
#
#
from smbus import SMBus
import time

ALLEGRO_ADD = 0x61

#Change this value to desired address
ALLEGRO_NEW_ADD = 0x62

ACCESS_CODE_SET = 0x2F

CUSTOMER_ACCESS_SET = 0x30

I2C_SLV_ADDR_SET = 0xF

CUSTOMER_CODE = 0x4F70656E

i2cbus = SMBus(1)

#convert desired address to little endian with set bit
new_addr_temp = ((ALLEGRO_NEW_ADD << 2) | 0x3) + 512
new_addr = [new_addr_temp & 0xFF, (new_addr_temp >> 8) & 0xFF, (new_addr_temp >> 16) & 0xFF, (new_addr_temp >> 24) & 0xFF]

#convert customer code to little endian
code = [CUSTOMER_CODE & 0xFF, (CUSTOMER_CODE >> 8) & 0xFF, (CUSTOMER_CODE >> 16) & 0xFF, (CUSTOMER_CODE >> 24) & 0xFF]
i2cbus.write_i2c_block_data(ALLEGRO_ADD, ACCESS_CODE_SET, code)
time.sleep(.5)
is_set = i2cbus.read_byte_data(ALLEGRO_ADD, CUSTOMER_ACCESS_SET)
time.sleep(.5)

if is_set == 1:
	print("in programming mode")


	i2cbus.write_i2c_block_data(ALLEGRO_ADD, I2C_SLV_ADDR_SET, new_addr)

	time.sleep(0.5)
	# Read the 32-bit register from the EEPROM
	data = i2cbus.read_i2c_block_data(ALLEGRO_ADD, I2C_SLV_ADDR_SET, 4)

	# Convert the bytes to an integer value (little-endian format)
	value = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]
	value1 = (new_addr[3] << 24) | (new_addr[2] << 16) | (new_addr[1] << 8) | new_addr[0]
	if value == value1:
		print("Success the address has been changed to 0x{:08X}".format(ALLEGRO_NEW_ADD))
		print("Value read from register 0x0F: 0x{:08X}".format(value))
	else:
		print("An error occur address sent does not match returned")

else:
	print("not in programming mode")
