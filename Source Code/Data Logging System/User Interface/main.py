#Author: Nicholas Altwasser
#SSID: 200389010
#
#
#
#

import power_monitor
import camera_test
import magnetic_sensor
import os
import serial
from queue import Empty
from multiprocessing import Process, Pipe
import multiprocessing
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.graphics import Color, Rectangle
from kivy.uix.button import Button
from kivy.lang import Builder
from kivy.garden.graph import Graph, MeshLinePlot, SmoothLinePlot
from kivy.graphics import Mesh
from kivy.graphics.vertex_instructions import Point

import glob
import time
from kivy.config import Config as KivyConfig
from kivy.properties import (
    NumericProperty, ReferenceListProperty, ObjectProperty
)
from kivy.vector import Vector
from kivy.clock import Clock

from kivy.core.window import Window
from kivy.graphics.vertex_instructions import Point
from kivy.graphics.context_instructions import Color
import csv

KivyConfig.set('graphics', 'fps', 100)
Window.maximize()


#Temp data input code
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')




class Temp_Sensor():
	def __init__(self):
		import os
		import glob
		os.system('modprobe w1-gpio')
		os.system('modprobe w1-therm')
		self.base_dir = '/sys/bus/w1/devices/'
		self.device1_folder = self.base_dir+'28-00000f1ab604'
		self.device2_folder = self.base_dir+'28-00000f1aac9e'
		self.device3_folder = self.base_dir+'28-00000f1abe26'
		self.device1_file = self.device1_folder + '/w1_slave'
		self.device2_file = self.device2_folder + '/w1_slave'
		self.device3_file = self.device3_folder + '/w1_slave'
		
		self.thread_enable = 1
		self.last_value_temp1 = 0
		self.last_value_temp2 = 0
		self.last_value_temp3 = 0	
	
	
	
	def read_temp(self, device_file):
		
		lines = self.read_temp_raw(device_file)
		if len(lines) > 0:
			while lines[0].strip()[-3:] != 'YES':
				time.sleep(0.2)
				lines = self.read_temp_raw(device_file)
			equals_pos = lines[1].find('t=')
			if equals_pos != -1:
				temp_string = lines[1][equals_pos+2:]
				temp_c = float(temp_string) / 1000.0
				
				return time.time(), temp_c
		else:
			return 0,0
	
	def read_temp_raw(self, device_file):
		try:
			
			f = open(device_file, 'r')
			lines = f.readlines()
			f.close()
			
			return lines
		except FileNotFoundError:
			print("Error Reading Device: "+device_file)
			
			lines = ['00 00 00 00 00 00 00 00 00 : crc=00 YES\n', '00 00 00 00 00 00 00 00 00 t=0\n']
			
			return lines
	
	def get_temp(self, queue_temp1, queue_temp2, queue_temp3, queue_time_temp1, queue_time_temp2, queue_time_temp3, queue_stop):
		
		while self.thread_enable:	
			
			time_temp_1, temp_1 = self.read_temp(self.device1_file)
			time_temp_2, temp_2 = self.read_temp(self.device2_file)
			time_temp_3, temp_3 = self.read_temp(self.device3_file)
			
			if temp_1 == 0:
				queue_temp1.put(self.last_value_temp1)
			else:
				self.last_value_temp1 = temp_1
				queue_temp1.put(temp_1)
				
			if temp_2 == 0:
				queue_temp2.put(self.last_value_temp2)
			else:
				self.last_value_temp2 = temp_2
				queue_temp2.put(temp_2)
			
			if temp_3 == 0:
				queue_temp3.put(self.last_value_temp3)
			else:
				self.last_value_temp3 = temp_3
				queue_temp3.put(temp_3)
			
			
			queue_time_temp1.put(time_temp_1)
			
			
			queue_time_temp2.put(time_temp_2)
			
			
			queue_time_temp3.put(time_temp_3)

			try:
				if queue_stop.get_nowait() == 1:
					self.thread_enable = 0
			except:
				pass
		
	

# End Temp data input code
		
class Graphs(BoxLayout):		
	def __init__(self, **kwargs):
		
		super(Graphs, self).__init__(**kwargs)
		
		
		#Enable Serial
		self.ser = serial.Serial(
			port ='/dev/serial0',
			baudrate = 9600,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=1)
		self.root = BoxLayout(orientation='vertical')
		
		# declare plots for coil temperatures
		self.plot_temp1 = MeshLinePlot(color=[1, 0, 0, 1])
		self.plot_temp2 = MeshLinePlot(color=[0, 1, 0, 1])
		self.plot_temp3 = MeshLinePlot(color=[0, 0, 1, 1])
		
		# declare plots for coil voltages
		self.plot_volt1 = MeshLinePlot(color=[1, 0, 0, 1])
		self.plot_volt2 = MeshLinePlot(color=[0, 1, 0, 1])
		self.plot_volt3 = MeshLinePlot(color=[0, 0, 1, 1])
		
		# declare plots for coil currents
		self.plot_curr1 = MeshLinePlot(color=[1, 0, 0, 1])
		self.plot_curr2 = MeshLinePlot(color=[0, 1, 0, 1])
		self.plot_curr3 = MeshLinePlot(color=[0, 0, 1, 1])
		
		self.plot_ball = MeshLinePlot(color=[1, 0, 0, 1])
		
		self.plot_magx = MeshLinePlot(color=[1, 0, 0, 1])
		self.plot_magy = MeshLinePlot(color=[0, 1, 0, 1])
		self.plot_magz = MeshLinePlot(color=[0, 0, 1, 1])
		
		
		self.cols = 1
		
		# create grid layout for buttons and text boxes
		self.control_layout = GridLayout(cols=2, row_force_default=True, row_default_height=40)
		
		self.start_w = Button(text='Start', on_press=self.start)
		self.control_layout.add_widget(self.start_w)
		
		self.stop_w = Button(text='Stop', on_press=self.stop)
		self.control_layout.add_widget(self.stop_w)
		
		self.max_temp_label = Label(text="Max Coil Temp")
		self.control_layout.add_widget(self.max_temp_label )
		self.max_temp = TextInput(multiline=False)
		self.control_layout.add_widget(self.max_temp)
		
		self.x_pos_d_label = Label(text="X Position Desired:")
		self.control_layout.add_widget(self.x_pos_d_label)
		self.x_pos_d = TextInput(multiline=False)
		self.control_layout.add_widget(self.x_pos_d)
		
		self.y_pos_d_label = Label(text="Y Position Desired:")
		self.control_layout.add_widget(self.y_pos_d_label)
		self.y_pos_d = TextInput(multiline=False)
		self.control_layout.add_widget(self.y_pos_d)
		
		self.x_pos_c_label = Label(text="X Position Current: ")
		self.control_layout.add_widget(self.x_pos_c_label)
		
		self.y_pos_c_label = Label(text="Y Position Current: ")
		self.control_layout.add_widget(self.y_pos_c_label)
		
		self.submit_w = Button(text='Submit', on_press=self.submit)
		self.control_layout.add_widget(self.submit_w)
		
		# add control layout to root layout
		self.root.add_widget(self.control_layout)
		
		# create grid layout for position graph
		self.position_layout = GridLayout(cols = 1)
		
		self.magnetic_layout = GridLayout(cols = 1)
		
		#initalize magnetic sensor graph
		self.magnetic_graph = Graph(
				xlabel='Data Points',
				ylabel='mT',
				x_ticks_minor=0,
				x_ticks_major=50,
				y_ticks_major=10,
				y_ticks_minor=0,
				y_grid_label=True,
				x_grid_label=True,
				padding=5,
				xlog=False,
				ylog=False,
				xmin=0,
				ymin=-30,
				ymax=30,
				xmax=100,
				size=(800, 275),
				size_hint=(None, None))
				
		
		self.magnetic_graph.add_plot(self.plot_magx)
		self.magnetic_graph.add_plot(self.plot_magy)
		self.magnetic_graph.add_plot(self.plot_magz)
			
		self.magnetic_layout.add_widget(self.magnetic_graph)  
		# initialize position graph
		self.position_graph = Graph(
				xlabel='x',
				ylabel='y',
				x_ticks_minor=2,
				x_ticks_major=10,
				y_ticks_major=10,
				y_ticks_minor=2,
				y_grid_label=True,
				x_grid_label=True,
				padding=5,
				xlog=False,
				ylog=False,
				xmin=0,
				ymin=0,
				ymax=42,
				xmax=58,
				size=(800, 275),
				size_hint=(None, None))
		
		
		self.position_graph.add_plot(self.plot_ball)
		
		self.position_layout.add_widget(self.position_graph)
		with self.position_graph.canvas:
			Color(1, 0, 0, 1)
			self.plot_ball = Point(pointsize=10)
		
		
		
		# add position layout to root layout
		self.root.add_widget(self.magnetic_layout)
		self.root.add_widget(self.position_layout)
		
		
		# create box layout for voltage and current graphs
		self.graph_layout = BoxLayout(orientation='vertical', minimum_height=100, minimum_width=1000)
		
		#Initializie Voltage Graph
		self.graph_voltage = Graph(
					xlabel='Data Points',
					ylabel='Voltage (V)',
					x_ticks_minor=0,
					x_ticks_major=50,
					y_ticks_major=5,
					y_ticks_minor=1,
					y_grid_label=True,
					x_grid_label=True,
					padding=5,
					xlog=False,
					ylog=False,
					xmin=0,
					ymin=-26, ymax = 26, 
					size = (800,275), 
					size_hint=(None,None))
			
		self.graph_voltage.add_plot(self.plot_volt1)
		self.graph_voltage.add_plot(self.plot_volt2)
		self.graph_voltage.add_plot(self.plot_volt3)
			
		self.graph_layout.add_widget(self.graph_voltage)
			
			#Initializie Current Graph
		self.graph_current = Graph(
						xlabel='Data Points',
						ylabel='Current (A)',
						x_ticks_minor=0,
						x_ticks_major=50,
						y_ticks_major=3,
						y_ticks_minor=1,
						y_grid_label=True,
						x_grid_label=True,
						padding=5,
						xlog=False,
						ylog=False,
						xmin=0,
						ymin=-10, ymax = 10, 
						size = (800,275), 
						size_hint=(None,None))
			
		self.graph_current.add_plot(self.plot_curr1)
		self.graph_current.add_plot(self.plot_curr2)
		self.graph_current.add_plot(self.plot_curr3)
			
		self.graph_layout.add_widget(self.graph_current)     
			
		#Initialize Temp Graph
		self.graph_temp = Graph(
								xlabel='Data Points',
								ylabel='Temperature (C)',
								x_ticks_minor=0,
								x_ticks_major=50,
								y_ticks_major=5,
								y_grid_label=True,
								x_grid_label=True,
								padding=5,
								xlog=False,
								ylog=False,
								xmin=0,
								ymin=0, 
								size = (800,275), 
								size_hint=(None,None))
			
		self.graph_temp.add_plot(self.plot_temp1)
		self.graph_temp.add_plot(self.plot_temp2)
		self.graph_temp.add_plot(self.plot_temp3)
			
		self.graph_layout.add_widget(self.graph_temp)
		self.add_widget(self.root)
		self.add_widget(self.graph_layout)
		
		self.clock_event = None
		
	def start(self, dt):
		
		#define start time of test
		self.start_time = time.time()
		
		self.get_temp_clock = Clock.schedule_interval(self.get_value_temp, 1)
		
		self.get_volt_curr_clock = Clock.schedule_interval(self.get_value_voltage, .75)	
		
		self.get_ball_clock = Clock.schedule_interval(self.get_point_ball, 1)
		
		self.get_mag_clock = Clock.schedule_interval(self.get_value_mag, 1)
		
		#Coil temperature lists
		self.plot_temp1.temp_x = [0,0]
		self.plot_temp1.temp_y1 = [0,0]
		self.plot_temp2.temp_y2 = [0,0]
		self.plot_temp3.temp_y3 = [0,0]
		
		#Coil voltage lists
		self.plot_volt1.volt_x = []
		self.plot_volt1.volt_y1 = []
		self.plot_volt2.volt_y2 = []
		self.plot_volt3.volt_y3 = []
		
		#Coil current Lists
		self.plot_curr1.curr_x = []
		self.plot_curr1.curr_y1 = []
		self.plot_curr2.curr_y2 = []
		self.plot_curr3.curr_y3 = []
		
		#magnetic Lists
		self.plot_magx.mag_xx = [0,0]
		self.plot_magx.magx = [0,0]
		self.plot_magy.magy = [0,0]
		self.plot_magz.magz = [0,0]
		
	
		self.count_temp = 1
		self.count_volt = 0
		self.count_curr = 0
		self.count_mag = 1
		
		
		#Process temp sensor data input
		self.queue_temp1 = multiprocessing.Queue()
		self.queue_temp2 = multiprocessing.Queue()
		self.queue_temp3 = multiprocessing.Queue()
		
		self.queue_time_temp1 = multiprocessing.Queue()
		self.queue_time_temp2 = multiprocessing.Queue()
		self.queue_time_temp3 = multiprocessing.Queue()

		self.queue_stop_temp = multiprocessing.Queue()
		
		self.temp = Temp_Sensor()
		self.T1 = Process(target= self.temp.get_temp, args=(self.queue_temp1,
		self.queue_temp2, self.queue_temp3,
		self.queue_time_temp1,
		self.queue_time_temp2,
		self.queue_time_temp3, self.queue_stop_temp,), daemon = True)
		self.T1.start()
		
		#Process voltage and current sensor data input
		self.queue_volt1 = multiprocessing.Queue()
		self.queue_volt2 = multiprocessing.Queue()
		self.queue_volt3 = multiprocessing.Queue()
		self.queue_curr1 = multiprocessing.Queue()
		self.queue_curr2 = multiprocessing.Queue()
		self.queue_curr3 = multiprocessing.Queue()
		self.queue_time1 = multiprocessing.Queue()
		self.queue_time2 = multiprocessing.Queue()
		self.queue_time3 = multiprocessing.Queue()
		self.queue_stop_volt = multiprocessing.Queue()
		self.lock_t2 = multiprocessing.Lock()
		self.volt_curr = power_monitor.volt_current()
		self.T2 = Process(target= self.volt_curr.get_volt_curr, args=(self.queue_volt1,
		self.queue_volt2,
		self.queue_volt3,
		self.queue_curr1,
		self.queue_curr2,
		self.queue_curr3,
		self.queue_time1,
		self.queue_time2,
		self.queue_time3, self.lock_t2, self.queue_stop_volt,), daemon = True)
		self.T2.start()
		
		#Multiproccess ball camera tracking input
		self.queue_x = multiprocessing.Queue()
		self.queue_y = multiprocessing.Queue()
		self.queue_time_pos = multiprocessing.Queue()

		self.queue_stop_ball = multiprocessing.Queue()
		
		self.ball = camera_test.camera_tracking()
		self.T3 = Process(target= self.ball.position, args=(self.queue_x, self.queue_y,self.queue_time_pos,self.queue_stop_ball,), daemon = True)
		self.T3.start()
		
		#process mag sensor data input
		self.queue_magx = multiprocessing.Queue()
		self.queue_magy = multiprocessing.Queue()
		self.queue_magz = multiprocessing.Queue()
		self.queue_time_magx = multiprocessing.Queue()
		self.queue_time_magy = multiprocessing.Queue()
		self.queue_time_magz = multiprocessing.Queue()
		
		self.queue_stop_mag = multiprocessing.Queue()
		
		self.mag = magnetic_sensor.Mag_Sensor()
		self.T4 = Process(target= self.mag.mag_xyz, args=(self.queue_magx, self.queue_magy, self.queue_magz, self.queue_time_magx, self.queue_time_magy, self.queue_time_magz, self.queue_stop_mag,), daemon = True)
		self.T4.start()
		
		#Tell pi pico to start
		self.ser.write(b'start\r\n')
		
	def stop(self, dt):
		#get stop time
		self.stop_time = time.time()
		
		#Tell pi pico to stop
		self.ser.write(b'stop\r\n')
		
		#Cancel clocks that update graphs and files
		self.get_temp_clock.cancel()
		
		self.get_volt_curr_clock.cancel()
		
		self.get_ball_clock.cancel()
		
		self.get_mag_clock.cancel()
		
		#stop all spawned processes
		self.queue_stop_temp.put(1)
		self.queue_stop_volt.put(1)
		self.queue_stop_ball.put(1)
		self.queue_stop_mag.put(1)
	
		
	def submit(self, dt):
		
		#Take in Max coil Temp
		self.max_temp_value=self.max_temp.text
		self.max_temp_label.text = "Max Coil Temp: " + self.max_temp_value
		self.max_temp_value = int(self.max_temp_value)
		print(self.max_temp_value)
		
		#Take in desired X position
		self.x_pos_d_value = self.x_pos_d.text
		self.x_pos_d_label.text = "Position Desired X: " + self.x_pos_d_value
		
		#Take in desired Y Position
		self.y_pos_d_value = self.y_pos_d.text
		self.y_pos_d_label.text = "Position Desired Y: " + self.y_pos_d_value
		
		#Send desired postions to pico
		msg = "x{}y{}\r\n".format(self.x_pos_d_value,self.y_pos_d_value)
		self.ser.write(msg.encode())
		
	def update_point_ball(self, ball_x1, ball_y1):
		
		#ball_x1 = 58
		#ball_y1 = 0
        #Scale the x and y coordinates to fit within the graph dimensions
		self.plot_ball.points = [(ball_x1*736/58)+50, (ball_y1*207/42)+50]
		
		
		
		#Send current ball position to pi Pico
		msg = "xc{}yc{}\r\n".format(ball_x1,ball_y1)
		self.ser.write(msg.encode())
		
	def get_point_ball(self,dt):
		
		try:
			ball_x1 = self.queue_x.get_nowait()
		except Empty:
			ball_x1 = 0
		try:
			ball_y1 = self.queue_y.get_nowait()
		except Empty:
			ball_y1 = 0
		try:
			time_pos = self.queue_time_pos.get_nowait()
			time_pos -= self.start_time
		except Empty:
			time_pos = 0
		
		with open('position_{}.csv'.format(self.start_time), 'a', newline='') as file:
			
			writer = csv.writer(file)	
			
			#write the newly taken in data to the position log file
			writer.writerow([time_pos, ball_x1, ball_y1])
		
		self.x_pos_c_label.text = "X Position Current: " + str(ball_x1)
		
		self.y_pos_c_label.text = "Y Position Current: "+  str(ball_y1)
		
		self.update_point_ball(ball_x1, ball_y1)
		
	def update_points_voltage(self):
		
		new_volt_y1 = self.plot_volt1.volt_y1
		new_volt_y2 = self.plot_volt2.volt_y2
		new_volt_y3 = self.plot_volt3.volt_y3
		
		new_curr_y1 = self.plot_curr1.curr_y1
		new_curr_y2 = self.plot_curr2.curr_y2
		new_curr_y3 = self.plot_curr3.curr_y3
		
		new_volt_x = self.plot_volt1.volt_x
		new_curr_x = self.plot_curr1.curr_x
		
		with open('voltage_{}.csv'.format(self.start_time), 'a', newline='') as file:
			
			writer = csv.writer(file)
			
			while not self.queue_curr3.empty():
				self.lock_t2.acquire()
				
				
				try:
					new_volt_y1.append(self.queue_volt1.get_nowait())
				except Empty:
					new_volt_y1.append(0)
					
				try:
					new_volt_y2.append(self.queue_volt2.get_nowait())
				except Empty:
					new_volt_y2.append(0)
					
				try:
					new_volt_y3.append(self.queue_volt3.get_nowait())
				except Empty:
					new_volt_y3.append(0)
					
				try:
					new_curr_y1.append(self.queue_curr1.get_nowait())
				except Empty:
					new_curr_y1.append(0)
					
				try:
					new_curr_y2.append(self.queue_curr2.get_nowait())
				except Empty:
					new_curr_y2.append(0)
					
				try:
					new_curr_y3.append(self.queue_curr3.get_nowait())
				except Empty:
					new_curr_y3.append(0)
					
				
				
				self.count_volt += 1
				
				try:
					time_temp1 = self.queue_time1.get_nowait()
					time_temp1 -= self.start_time
				except Empty:
					time_temp1 = 0
					pass
				try:
					time_temp2 = self.queue_time2.get_nowait()
					time_temp2 -= self.start_time
				except Empty:
					time_temp2 = 0
					pass
				try:
					time_temp3 = self.queue_time3.get_nowait()
					time_temp3 -= self.start_time
				except Empty:
					time_temp3 = 0
					pass
				self.lock_t2.release()
				
				if not new_volt_x:
					new_volt_x.append(0)
				else:
					new_volt_x.append(new_volt_x[-1] + 1)
			
				if self.count_volt > 100:
					new_volt_y1.pop(0)
					new_volt_y2.pop(0)
					new_volt_y3.pop(0)
				
					new_curr_y1.pop(0)
					new_curr_y2.pop(0)
					new_curr_y3.pop(0)
					new_volt_x.pop(0)
					
					writer.writerow([time_temp1, new_volt_y1[99], new_curr_y1[99], time_temp2, new_volt_y2[99], new_curr_y2[99],  time_temp3, new_volt_y3[99], new_curr_y3[99]])
				else:
					writer.writerow([time_temp1, new_volt_y1[self.count_volt-1] , new_curr_y1[self.count_volt-1], time_temp2, new_volt_y2[self.count_volt-1], new_curr_y2[self.count_volt-1], time_temp3, new_volt_y3[self.count_volt-1], new_curr_y3[self.count_volt-1]])
		
		
		self.plot_curr1.points= list(zip(new_volt_x, new_curr_y1))
		self.plot_curr2.points= list(zip(new_volt_x, new_curr_y2))
		self.plot_curr3.points= list(zip(new_volt_x, new_curr_y3))
		
		self.plot_volt1.points= list(zip(new_volt_x, new_volt_y1))
		self.plot_volt2.points= list(zip(new_volt_x, new_volt_y2))
		self.plot_volt3.points= list(zip(new_volt_x, new_volt_y3))
		
		if len(new_volt_y3):
			self.update_axis_voltage()
	
	def get_value_voltage(self, dt):
		
		self.update_points_voltage()
		
	def update_axis_voltage(self):
		
		self.graph_voltage.xmax = self.count_volt
		self.graph_voltage.xmin = 0
		if self.count_volt > 100:
			self.graph_voltage.xmin = self.count_volt - 100
		
		#find max of all values
		max1=max(self.plot_volt1.volt_y1)+2
		max2=max(self.plot_volt2.volt_y2)+2
		max3=max(self.plot_volt3.volt_y3)+2
		
		if max1 >= max2 and max1 >= max3:
			self.graph_voltage.ymax = max1
		
		if max2 >= max1 and max2 >= max3:
			self.graph_voltage.ymax = max2
		
		if max3 >= max1 and max3>= max2:
			self.graph_voltage.ymax = max3
		
		self.graph_current.xmax = self.count_volt
		self.graph_current.xmin = 0
		if self.count_volt > 100:
			self.graph_current.xmin = self.count_volt - 100
		
		#find max of all values
		
		max1=max(self.plot_curr1.curr_y1)+2
		max2=max(self.plot_curr2.curr_y2)+2
		max3=max(self.plot_curr3.curr_y3)+2
		
		if max1 >= max2 and max1 >= max3:
			self.graph_current.ymax = max1
		
		if max2 >= max1 and max2 >= max3:
			self.graph_current.ymax = max2
		
		if max3 >= max1 and max3>= max2:
			self.graph_current.ymax = max3
		
	
	def update_points_temp(self):
		
		new_temp_y1 = self.plot_temp1.temp_y1
		new_temp_y2 = self.plot_temp2.temp_y2
		new_temp_y3 = self.plot_temp3.temp_y3
		new_temp_x = self.plot_temp1.temp_x
		
		with open('Coil Temperature_{}.csv'.format(self.start_time), 'a', newline='') as file:
			
			writer = csv.writer(file)
		
			while not self.queue_temp1.empty():
				try:
					temp_1 = self.queue_temp1.get_nowait()
				except Empty:
					temp_1 = self.plot_temp1.temp_y1[self.count_temp-1]
					pass
				try:
					temp_2 = self.queue_temp2.get_nowait()
				except Empty:
					temp_2 = self.plot_temp2.temp_y2[self.count_temp-1]
					pass
				try:
					temp_3 = self.queue_temp3.get_nowait()
				except Empty:
					temp_3 = self.plot_temp3.temp_y3[self.count_temp-1]
					pass
					
				try:
					time_temp1 = self.queue_time_temp1.get_nowait()
					time_temp1 -= self.start_time
				except Empty:
					time_temp1 = 0
					pass
				try:
					time_temp2 = self.queue_time_temp2.get_nowait()
					time_temp2 -= self.start_time
				except Empty:
					time_temp2 = 0
					pass
				try:
					time_temp3 = self.queue_time_temp3.get_nowait()
					time_temp3 -= self.start_time
				except Empty:
					time_temp3 = 0
					pass
					
				
				new_temp_y1.append(temp_1)
				new_temp_y2.append(temp_2)
				new_temp_y3.append(temp_3)
				new_temp_x.append(self.count_temp)
				
				if self.count_temp > 100:
					new_temp_y1.pop(0)
					new_temp_y2.pop(0)
					new_temp_y3.pop(0)
					new_temp_x.pop(0)
				
				self.count_temp += 1
				
				#write the newly taken in data to the temperature log file
				writer.writerow([time_temp1, temp_1, time_temp2, temp_2, time_temp3, temp_3])
				
		self.plot_temp1.points = list(zip(new_temp_x, new_temp_y1))
		self.plot_temp2.points = list(zip(new_temp_x, new_temp_y2))
		self.plot_temp3.points = list(zip(new_temp_x, new_temp_y3))
		
		self.update_axis_temp()
	
	def get_value_temp(self, dt):
		
		self.update_points_temp()
		
	def update_axis_temp(self):
		self.graph_temp.xmax = self.count_temp
		self.graph_temp.xmin = 0
		if self.count_temp > 100:
			self.graph_temp.xmin = self.count_temp - 100
		
		
		max1= max(self.plot_temp1.temp_y1)+2
		max2= max(self.plot_temp2.temp_y2)+2
		max3= max(self.plot_temp3.temp_y3)+2
		if (max1-2) > self.max_temp_value or (max2-2) > self.max_temp_value or (max3-2) > self.max_temp_value:
			
			#Add write to log file syaing test stopped due to overtemp
			self.stop(0)
			
		if max1 >= max2 and max1 >= max3:
			self.graph_temp.ymax = max1
		
		if max2 >= max1 and max2 >= max3:
			self.graph_temp.ymax = max2
		
		if max3 >= max1 and max3>= max2:
			self.graph_temp.ymax = max3
	
	def get_value_mag(self, dt):
		self.update_points_mag()
		
	def update_points_mag(self):
		
		new_mag_x = self.plot_magx.magx
		new_mag_y = self.plot_magy.magy
		new_mag_z = self.plot_magz.magz
		new_mag_xx = self.plot_magx.mag_xx
		
		with open('Magnetic Sensor_{}.csv'.format(self.start_time), 'a', newline='') as file:
			
			writer = csv.writer(file)
			while not self.queue_magz.empty():
				try:
					mag_x = self.queue_magx.get_nowait()
				except Empty:
					mag_x = self.plot_magx.magx[self.count_mag-1]
					pass
				try:
					mag_y = self.queue_magy.get_nowait()
				except Empty:
					mag_y = self.plot_magy.magy[self.count_mag-1]
					pass
				try:
					mag_z = self.queue_magz.get_nowait()
				except Empty:
					mag_z = self.plot_magz.magz[self.count_mag-1]
					pass
					
				try:
					time_mag_x = self.queue_time_magx.get_nowait()
					time_mag_x -= self.start_time
				except Empty:
					time_mag_x = 0
					pass
				try:
					time_mag_y = self.queue_time_magy.get_nowait()
					time_mag_y -= self.start_time
				except Empty:
					time_mag_y = 0
					pass
				try:
					time_mag_z = self.queue_time_magz.get_nowait()
					time_mag_z -= self.start_time
				except Empty:
					time_mag_z = 0
					pass
				
				new_mag_x.append(mag_x)
				new_mag_y.append(mag_y)
				new_mag_z.append(mag_z)
				
				new_mag_xx.append(self.count_mag)
				if self.count_mag > 100:
					new_mag_x.pop(0)
					new_mag_y.pop(0)
					new_mag_z.pop(0)
					new_mag_xx.pop(0)
				
				self.count_mag += 1
				writer.writerow([time_mag_x, mag_x, time_mag_y, mag_y, time_mag_z, mag_z])
		
		self.plot_magx.points = list(zip(new_mag_xx, new_mag_x))
		self.plot_magy.points = list(zip(new_mag_xx, new_mag_y))
		self.plot_magz.points = list(zip(new_mag_xx, new_mag_z))
		
		self.update_axis_mag()
	
	def update_axis_mag(self):
		self.magnetic_graph.xmax = self.count_mag
		self.magnetic_graph.xmin = 0
		if self.count_mag > 100:
			self.magnetic_graph.xmin = self.count_mag - 100
		
		max1= max(self.plot_temp1.temp_y1)+2
		max2= max(self.plot_temp2.temp_y2)+2
		max3= max(self.plot_temp3.temp_y3)+2
		if max1 >= max2 and max1 >= max3:
			self.graph_temp.ymax = max1
		
		if max2 >= max1 and max2 >= max3:
			self.graph_temp.ymax = max2
		
		if max3 >= max1 and max3 >= max2:
			self.graph_temp.ymax = max3

class GuiApp(App):
	
	def build(self):
		return Graphs()

if __name__ == '__main__':
	GuiApp().run()

	
	
		


