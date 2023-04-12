#Author: Nicholas Altwasser
#SSID: 200389010
#
#
#
#
from smbus import SMBus
import time

MAGSENSOR_READ = 0x35
MAGSENSOR_WRITE = 0x35

X_REG_MSB = 0x10
X_REG_LSB = 0x11
Y_REG_MSB = 0x14
Y_REG_LSB = 0x15
Z_REG_MSB = 0x16
Z_REG_LSB = 0x17

XY_REG_LSB = 0x4
Z_REG_LSB = 0x5

i2cbus = SMBus(1)
try:
	i2cbus.write_byte_data(MAGSENSOR_READ, 0x0, 0x01)
	i2cbus.write_byte_data(MAGSENSOR_READ, 0x2, 0x70)
	i2cbus.write_byte_data(MAGSENSOR_READ, 0x1, 0x02)
except: 
	print("Sensor Disconnected")

class Mag_Sensor():
	def __init__(self):
		self.thread_enable = 1
	def read_XYZ(self):
		
		
		try:
			
			data_x_M = i2cbus.read_byte_data(MAGSENSOR_READ, X_REG_MSB)
			data_x_L = i2cbus.read_byte_data(MAGSENSOR_READ, X_REG_LSB)
			time_x = time.time()
		
		except:
			time_x = time.time()
			print("failed")
			data_x_M = 0xFF
			data_x_L = 0xFF
			
		try:
			
			data_y_M = i2cbus.read_byte_data(MAGSENSOR_READ, Y_REG_MSB)
			data_y_L = i2cbus.read_byte_data(MAGSENSOR_READ, Y_REG_LSB)
			time_y = time.time()
		except:
			time_y = time.time()
			print("failed")
			data_y_M = 0xFF
			data_y_L = 0xFF
			pass
		try:
			
			data_z_M = i2cbus.read_byte_data(MAGSENSOR_READ, Z_REG_MSB)
			data_z_L = i2cbus.read_byte_data(MAGSENSOR_READ, Z_REG_LSB)
			time_z = time.time()
		except:
			time_z = time.time()
			print("failed")
			data_z_M= 0xFF
			data_z_L= 0xFF
			pass
		
		
		data_X = ((data_x_M << 8) | data_x_L)
		data_Y = ((data_y_M << 8) | data_y_L)
		data_Z = ((data_z_M << 8) | data_z_L)
		
		
		return time_x, data_X, time_y, data_Y, time_z, data_Z
		#(data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]
	

	def mag_xyz(self, queue_magx, queue_magy, queue_magz, queue_time_magx, queue_time_magy, queue_time_magz, queue_stop):
			
			while self.thread_enable:
				(time_x, x, time_y, y, time_z, z) = self.read_XYZ()
				
				x = -(x & 0x8000) | (x & 0x7fff)
				b_x = ((x/32768)*133)
				
				y = -(y & 0x8000) | (y & 0x7fff)
				b_y = ((y/32768)*133)
				
				z = -(z & 0x8000) | (z & 0x7fff)
				b_z = ((z/32768)*133)
		
	
				queue_magx.put(b_x)
				queue_magy.put(b_y)
				queue_magz.put(b_z)
				queue_time_magx.put(time_x)
				queue_time_magy.put(time_y)
				queue_time_magz.put(time_z)
				
				try:
					if queue_stop.get_nowait() == 1:
						self.thread_enable = 0
				except:
					pass
				#instability noticed in the sensor when time is less than 0.5 seconds might have to add locks
				time.sleep(0.01)

		

