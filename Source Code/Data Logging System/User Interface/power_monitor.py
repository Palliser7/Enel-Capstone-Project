#Author: Nicholas Altwasser
#SSID: 200389010
#
#
#
#
from smbus import SMBus
import time
from time import sleep
from threading import Thread
import csv

ALLEGRO_1 = 0x60
ALLEGRO_2 = 0x61
ALLEGRO_3 = 0x62

#ALLEGRO REGISTER CONSSTANTS
#Write EEPROM
EEPROM_SET = 0x0B

RMS_AVERAGE_SET = 0x0C

PHASE_DELAY_SET = 0x0D

OVERVOLT_UNDERVOLT_SET = 0x0E

I2C_SLV_ADDR_SET = 0xF

ACCESS_CODE_SET = 0x2F

CUSTOMER_ACCESS_SET = 0x30

CUSTOMER_CODE = 0x4F70656E

#READ VOLATILE

VRMS_IRMS_GET = 0x20

POWER_FACTOR_GET = 0x22

NUMPTSOUT_GET = 0x25

VRMS_AVG_ONESEC_GET = 0x26

VRMS_AVG_ONEMIN_GET = 0x27

PACT_AVG_ONESEC_GET = 0x28

PACT_AVG_ONEMIN_GET = 0x29

INST_V_I_GET = 0x2A

INST_P_GET = 0x2C

FAULT_GET = 0x2D

VIN_MAX = 0.84

VIN_MIN = -0.84

FIXED_POINT_CONVERT = 32767
RSENSE_RATIO = 0.01264527755


i2cbus = SMBus(1)

class volt_current():	
	def __init__(self):
		self.thread_enable = 1
	def start_volt_current(self, dt):	
		Thread(target = self.get_volt_curr()).start()
	def read_volt_curr(self, ALLEGRO):
		
		try:
			data = i2cbus.read_i2c_block_data(ALLEGRO, INST_V_I_GET, 4)
			#CONVERT LITTLE ENDIAN TO BIT ENDIAN
			
			data_volt = (data[1] << 8) | data[0]
			data_curr = (data[3] << 8) | data[2]
		except:
			#print("read error device: ", hex(ALLEGRO))
			data_volt = 27500
			data_curr = 27500
		
	
		return (time.time(), data_volt, data_curr)
	
	
	def volt_curr(self, ALLEGRO):
		
		(time, volt_temp, curr_temp) = self.read_volt_curr(ALLEGRO)
		
	
		value_volt = -(volt_temp & 0x8000) | (volt_temp & 0x7fff)
		
	
		if value_volt < 0 and value_volt >= -(1 << 15):
		
			
			volt = (((value_volt*0.84)/110)/1000)/RSENSE_RATIO
		
		else:
		
			
			volt = (((value_volt*0.84)/110)/1000)/RSENSE_RATIO
		
		
		value_curr = -(curr_temp & 0x8000) | (curr_temp & 0x7fff)
		
	
		if value_curr < 0 and value_curr >= -(1 << 15):
		
			curr = (value_curr*.84)/305.6
		
		else:
		
			curr = (value_curr*.84)/305.6


		return time, volt, curr
	
	def get_volt_curr(self, queue_volt1,
		queue_volt2,
		queue_volt3,
		queue_curr1,
		queue_curr2,
		queue_curr3,
		queue_time1,
		queue_time2,
		queue_time3,lock_t2, queue_stop):
		
		while self.thread_enable:	
			
			(time_1, new_volt_1, new_curr_1) = self.volt_curr(ALLEGRO_1)
			sleep(0.0001)
			(time_2, new_volt_2, new_curr_2) = self.volt_curr(ALLEGRO_2)
			sleep(0.0001)
			(time_3, new_volt_3, new_curr_3) = self.volt_curr(ALLEGRO_3)
			sleep(0.0001)
			
			
			
			
			lock_t2.acquire()
			if not queue_volt1.full():
				queue_volt1.put_nowait(round(new_volt_1,4))
				queue_volt2.put_nowait(round(new_volt_2,4))
				queue_volt3.put_nowait(round(new_volt_3,4))
				queue_curr1.put_nowait(round(new_curr_1,4))
				queue_curr2.put_nowait(round(new_curr_2,4))
				queue_curr3.put_nowait(round(new_curr_3,4))
				queue_time1.put_nowait(round(time_1,4))
				queue_time2.put_nowait(round(time_2,4))
				queue_time3.put_nowait(round(time_3,4))
			lock_t2.release()
			try:
				if queue_stop.get_nowait() == 1:
					self.thread_enable = 0
			except:
				pass
			sleep(0.004)


	
	


	
