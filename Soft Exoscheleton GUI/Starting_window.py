from PyQt5 import QtCore, uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtCore import QCoreApplication
from serial.tools.list_ports import comports
# from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import numpy as np
import serial
import sys
import time
import datetime
import queue
import os
import re
import math 
from ctypes import c_bool
import multiprocessing as mp
from threading import Thread
from pglive.kwargs import Axis
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_axis_range import LiveAxisRange

# CALIBRATION VARIABLES
DATA_NUMBER = 37  # number of elements in string passed by arduino (last n° in Print2Serial + 1)
DEBUG = False  # debug mode (prints in terminal)
VISIBLE_PLOT_POINTS = 250
UPDATE_PLOT_FREQUENCY = 10  # was 20
SERIAL_BAUDRATE = 9600

## --------------------------------------------------------------------------------------------- ##
## -------------------------------------- STARTING WINDOW -------------------------------------- ##
## --------------------------------------------------------------------------------------------- ##

class Start_win(QMainWindow):
	def __init__(self,*args,**kwargs):
		super().__init__(*args,**kwargs)
		uic.loadUi('Starting_window.ui',self)

		# listing com ports in the box of the gui
		for port in comports():
			self.COMBox.addItem(port.name)
		
		# time-based loop -> checks if new COM is opened
		self.checkPlug_timer = QtCore.QTimer()
		self.checkPlug_timer.setInterval(10)
		self.checkPlug_timer.timeout.connect(self.checkPlug)
		self.checkPlug_timer.start()

		# time-based loop -> checks if all inputs are ok to start
		self.checkLaunch_timer = QtCore.QTimer()
		self.checkLaunch_timer.setInterval(10)
		self.checkLaunch_timer.timeout.connect(self.checkLaunch)

		# connecting buttons
		self.startButton.clicked.connect(self.getCOM)

		# listing current com ports
		self.starting_ports = []
		for port in comports():
			self.starting_ports.append(port.name)

	def checkLaunch(self):  # checks if objects are meaningful, if not border of objects becomes red
		if self.checkLaunch_timer.isActive():
			check_obj = []
		else:
			check_obj = [self.COMBox]
			
		if len(check_obj) != 0:
			for object in check_obj:
				if object.currentText() == '-':
					object.setStyleSheet('border: 2px solid red')
				else:
					if self.checkLaunch_timer.isActive():  # if checkLaunch is started by device plugged, the signal waiting timer has to be stopped 
						self.checkLaunch_timer.stop()
					self.launchMain()
		else:
			if self.checkLaunch_timer.isActive():  # if checkLaunch is started by device plugged, the signal waiting timer has to be stopped 
				self.checkLaunch_timer.stop()
			self.launchMain()

	def getCOM(self):
		self.PORT_name = self.COMBox.currentText()
		self.checkLaunch()
		self.checkPlug_timer.stop()  # stop checking if new COM port will be opened

	def checkPlug(self):
		# Storing active ports in a vector and in the gui's comboBox
		self.new_ports = []
		for port in comports():
			self.new_ports.append(port.name)
		if len(self.starting_ports) < len(self.new_ports):
			self.checkPlug_timer.stop()  # stop checking if new COM port will be opened
			changed_ports = list(set(self.new_ports).difference(set(self.starting_ports)))
			self.PORT_name = changed_ports[0]  # COM port plugged
			time.sleep(0.5)  # Waiting for the device manager list to update after plugging
			self.checkLaunch_timer.start()  # starting this timer = waiting for a signal to be chosen
		elif len(self.starting_ports) > len(self.new_ports):
			self.starting_ports = self.new_ports
		
	def launchMain(self):
		self.gui_main = main_window(self.PORT_name)
		if not DEBUG:
			self.gui_main.showMaximized()
		else:
			self.gui_main.show()
		self.close()


class main_window(QMainWindow):

	def __init__(self, PORT_name, *args, **kwargs):
		super(main_window, self).__init__(*args, **kwargs)
		self.ui = uic.loadUi('gui.ui', self) # load user interface
		self.ui.closeEvent = self.closeEvent

		# multiprocessing variables
		self.filename_queue = mp.Queue()
		self.writer_data_queue = mp.Queue()
		self.writer_kill = mp.Value(c_bool, False)
		self.serial_data_queue = mp.Queue()
		self.serial_kill = mp.Value(c_bool, False)
		self.serial_commands_queue = mp.Queue()
		self.flag_save_data = mp.Value(c_bool, False)
		self.update_plot_kill = mp.Value(c_bool, False)
		self.update_gui_queue = mp.Queue()

		# additional variables
		self.timer_reset = 0
		self.timer_five = True

		# queues for plot update
		# --> EMG
		self.emg1_queue = mp.Queue()
		self.emg2_queue = mp.Queue()
		# --> IMU
		self.torso_queue = mp.Queue()
		self.elevComp_queue = mp.Queue()
		self.flex_queue = mp.Queue()
		# --> SPEED & ACCELERATION
		self.speed_queue = mp.Queue()
		self.accel_queue = mp.Queue()
		# --> PRESSURE
		self.press1_queue = mp.Queue()

		# variables once @ the start
		self.COM_name_queue = mp.Queue()
		self.COM_name_queue.put(PORT_name)

		# opening serial communication + initializating input variables
		self.serial_process = mp.Process(target=main_window.serial_port_handler, args=(self.COM_name_queue, self.serial_commands_queue, self.serial_data_queue, self.serial_kill, self.flag_save_data, self.writer_data_queue))
		self.serial_process.start()

		# process for data writing on .csv file
		self.writer_process = mp.Process(target=main_window.data_writer, args=(self.flag_save_data, self.filename_queue, self.writer_data_queue, self.writer_kill))
		self.writer_process.start()

		# buttons
		self.ss_timer_button.clicked.connect(self.ss_timer)
		self.save_data_button.clicked.connect(self.save_data)
		self.pumps_button.clicked.connect(lambda: self.send_2msg("1001","1002"))
		self.pump1_button.clicked.connect(lambda: self.send_msg("1001"))
		self.pump2_button.clicked.connect(lambda: self.send_msg("1002"))
		self.valve1_button.clicked.connect(lambda: self.send_msg("1003"))
		# self.valve2_button.clicked.connect(lambda: self.send_msg("1004"))  # uncomment if valve 2 is installed in the robot
		self.presscontrol_button.clicked.connect(lambda: self.send_msg("2001"))
		self.fatiguetest_button.clicked.connect(self.startFatigueTest)
		self.calibrateIMU_Button.clicked.connect(lambda: self.send_msg("5001"))
		self.zeroKg.clicked.connect(lambda: self.send_msg("8000"))
		self.oneKg.clicked.connect(lambda: self.send_msg("8001"))
		self.twoKg.clicked.connect(lambda: self.send_msg("8002"))

		# spinner
		self.press_spinbox.valueChanged.connect(lambda: self.setPressure(1))
		self.pressth_spinbox.valueChanged.connect(self.setErrorThreshold)
		self.pressFatigue_spinbox.valueChanged.connect(lambda: self.setPressure(2))
		self.cyclesFatigue_spinbox.valueChanged.connect(self.setCycles)
		self.EMGbaseline_spinbox.valueChanged.connect(self.setEMGbaseline)

		# led
		self.image1.setPixmap(QPixmap("img/red_led.png"))
		self.image2.setPixmap(QPixmap("img/red_led.png"))
		self.image3.setPixmap(QPixmap("img/red_led.png"))
		# self.image4.setPixmap(QPixmap("img/red_led.png"))  # uncomment inf valve 2 in installed in the robot
		self.led_write.setPixmap(QPixmap("img/red_led.png"))
		self.led_IMUcalibration.setPixmap(QPixmap("img/red_led.png"))

		# tab
		self.tabWidget.currentChanged.connect(lambda: self.send_msg("0000"))

		# -------------------- plot setup -------------------- #
		# EMG plotter
		self.ui.EMG_plot.setBackground('w')
		self.ui.EMG_plot.setLabel('left', 'Amplitude (-)')
		self.ui.EMG_plot.showGrid(x=False, y=True)
		self.ui.EMG_plot.y_range_controller = LiveAxisRange(fixed_range=[0, 5000])
		# IMU plotter
		self.ui.IMU_plot.setBackground('w')
		self.ui.IMU_plot.setLabel('left', 'Amplitude (-)')
		self.ui.IMU_plot.showGrid(x=False, y=True)
		self.ui.IMU_plot.y_range_controller = LiveAxisRange(fixed_range=[-10, 190])
		# SPEED & ACCELERATION plotter
		self.ui.SPEEDACC_plot.setBackground('w')
		self.ui.SPEEDACC_plot.setLabel('left', 'Amplitude (-)')
		self.ui.SPEEDACC_plot.showGrid(x=False, y=True)
		self.ui.SPEEDACC_plot.y_range_controller = LiveAxisRange(fixed_range=[-5, 20])
		# PRESSURE plotter
		self.ui.PRESS_plot.setBackground('w')
		self.ui.PRESS_plot.setLabel('left', 'Pressure (psi)')
		self.ui.PRESS_plot.showGrid(x=False, y=True)
		self.ui.PRESS_plot.y_range_controller = LiveAxisRange(fixed_range=[0, 27])

		# -------------------- Creating lines for plots -------------------- #
		# EMG
		self.emg1_live = LiveLinePlot(pen="blue")  # EMG flexion / Analog read A4 in main.ino
		self.emg2_live = LiveLinePlot(pen="red")  # EMG abduction / Analog read A5 in main.ino
		# IMU
		self.torso_live = LiveLinePlot(pen="blue")
		self.elevComp_live = LiveLinePlot(pen="green")
		self.flex_live = LiveLinePlot(pen="red")
		# SPEED & ACCELERATION
		self.speed_live = LiveLinePlot(pen="blue")
		self.accel_live = LiveLinePlot(pen="green")
		# PRESSURE
		self.press1_live = LiveLinePlot(pen="red")

		# -------------------- Linking Lines and Plots -------------------- #
		# --> EMG
		self.ui.EMG_plot.addItem(self.emg1_live)
		self.ui.EMG_plot.addItem(self.emg2_live)
		# --> IMU
		self.ui.IMU_plot.addItem(self.torso_live)
		self.ui.IMU_plot.addItem(self.elevComp_live)
		self.ui.IMU_plot.addItem(self.flex_live)
		# --> SPEED & ACCELERATION
		self.ui.SPEEDACC_plot.addItem(self.speed_live)
		self.ui.SPEEDACC_plot.addItem(self.accel_live)
		# --> PRESSURE
		self.ui.PRESS_plot.addItem(self.press1_live)

		# -------------------- Data connectors lines -------------------- #
		# --> EMG
		conn_emg1 = DataConnector(self.emg1_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=False)
		conn_emg2 = DataConnector(self.emg2_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=True)
		# --> IMU
		conn_torso = DataConnector(self.torso_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=False)
		conn_elevComp = DataConnector(self.elevComp_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=False)
		conn_flex = DataConnector(self.flex_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=True)
		# --> SPEED & ACCELERATION
		conn_speed = DataConnector(self.speed_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=False)
		conn_accel = DataConnector(self.accel_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=True)
		# --> PRESSURE
		conn_press1 = DataConnector(self.press1_live, max_points=VISIBLE_PLOT_POINTS, plot_rate=UPDATE_PLOT_FREQUENCY, ignore_auto_range=False)

		# -------------------- Thread generation -------------------- #
		# Spreading data into the right queues
		queue_spread_process = mp.Process(target=main_window.queue_spread, 
				    					  args=(self.serial_data_queue, 
		     									self.emg1_queue,
											    self.emg2_queue,
											    self.torso_queue,
											    self.elevComp_queue,
											    self.flex_queue,
											    self.speed_queue,
											    self.accel_queue,
											    self.press1_queue,
												self.update_gui_queue, self.update_plot_kill))
		queue_spread_process.start()
		
		## -------------------- 1 only thread managing 6 plots -------------------- ##
		plot_update_thread1 = Thread(target=main_window.data_generator_for4, args=(self.emg1_queue, conn_emg1,
										 										   self.emg2_queue, conn_emg2,
										 										   self.torso_queue, conn_torso,
																				   self.elevComp_queue, conn_elevComp))
		plot_update_thread1.start()
		plot_update_thread2 = Thread(target=main_window.data_generator_for4, args=(self.flex_queue, conn_flex,
																				   self.speed_queue, conn_speed,
																				   self.accel_queue, conn_accel,
																				   self.press1_queue, conn_press1))
		plot_update_thread2.start()

		# time-based loop
		self.timer = QtCore.QTimer()
		self.timer.timeout.connect(self.update_GUI)
		self.send_msg("0001") # reset arduino when GUI start
		self.timer.start()

	# ----------------------- FUNCTIONS

	# Function to generate a process that manages serial communication
	def serial_port_handler(ComName, serial_commands, serial_data_queue, serial_kill, flag_save_data, writer_data_queue):
		# Opening Serial communication
		COM_port = ComName.get()
		arduino = serial.Serial(COM_port, baudrate=SERIAL_BAUDRATE, timeout=.1)

		while serial_kill.value == False:
			if not serial_commands.empty():
				command = serial_commands.get_nowait()
				command = command  + '|'
				if DEBUG:
					print(f'Command received: {command.encode("utf-8")} command queue size: {serial_commands.qsize()}')
				arduino.write(command.encode("utf-8"))

			try:
				data = arduino.readline().decode("utf-8")
				values = data.split(",") # split at commas
				values = values[:-1] # remove final '/r/n'
				if len(values) != DATA_NUMBER:
					if DEBUG:
						print('bad data')
					continue
				if not DEBUG:
					print(values)
				serial_data_queue.put(values, False)
				if DEBUG:
					print(f'Serial queue size: {serial_data_queue.qsize()}')
				if flag_save_data.value == True:
					writer_data_queue.put(values, False)  # IMU calibration data removed 
			except (OSError, serial.SerialException):
				break
			except queue.Full:
				continue
			except Exception as e:
				print(e)
				break

		try:
			arduino.close()
		except Exception as e:
			print(e)

	# spreading data from serial into queues to update each plot and gui
	def queue_spread(serial, emg1, emg2, to, ele, fl, sp, acc, press1, gui_queue, update_plot_kill):
		while True:
			if update_plot_kill.value == True:
				break
			while not serial.empty():
				# getting data from serial queue
				try:
					data_from_serial = serial.get_nowait()
					# --> EMG
					emg1.put(data_from_serial[0])
					emg2.put(data_from_serial[2])
					# --> IMU
					to.put(data_from_serial[17])
					ele.put(data_from_serial[18])
					fl.put(data_from_serial[19])
					# --> SPEED & ACCELERATION
					sp.put(data_from_serial[8])  # shoulder IMU ang. speed
					acc.put(data_from_serial[9])  # shoulder IMU lin. accel.
					# --> PRESSURE
					press1.put(data_from_serial[20])

					# data to update gui
					gui_queue.put(data_from_serial)

					if DEBUG:
						print(f'EMG1 queue size: {emg1.qsize()} | EMG2 queue size: {emg2.qsize()}')
				except queue.Empty:
					continue

	# Thread for appending data to plot ########################################################################################################
	def data_generator_for8(data_queue1, connector1,
		    		   		data_queue2, connector2,
				       		data_queue3, connector3,
					   		data_queue4, connector4,
					   		data_queue5, connector5,
					   		data_queue6, connector6,
							data_queue7, connector7,
							data_queue8, connector8):
		x = [0, 0, 0, 0, 0, 0, 0, 0]
		while True:
			if not data_queue1.empty():
			# getting data from serial queue
				try:
					value = data_queue1.get_nowait()
					x[0] += 1
					connector1.cb_append_data_point(float(value), x[0])
				except queue.Empty:
					continue
			if not data_queue2.empty():
			# getting data from serial queue
				try:
					value = data_queue2.get_nowait()
					x[1] += 1
					connector2.cb_append_data_point(float(value), x[1])
				except queue.Empty:
					continue
			if not data_queue3.empty():
			# getting data from serial queue
				try:
					value = data_queue3.get_nowait()
					x[2] += 1
					connector3.cb_append_data_point(float(value), x[2])
				except queue.Empty:
					continue
			if not data_queue4.empty():
			# getting data from serial queue
				try:
					value = data_queue4.get_nowait()
					x[3] += 1
					connector4.cb_append_data_point(float(value), x[3])
				except queue.Empty:
					continue
			if not data_queue5.empty():
			# getting data from serial queue
				try:
					value = data_queue5.get_nowait()
					x[4] += 1
					connector5.cb_append_data_point(float(value), x[4])
				except queue.Empty:
					continue
			if not data_queue6.empty():
			# getting data from serial queue
				try:
					value = data_queue6.get_nowait()
					x[5] += 1
					connector6.cb_append_data_point(float(value), x[5])
				except queue.Empty:
					continue
			if not data_queue7.empty():
			# getting data from serial queue
				try:
					value = data_queue7.get_nowait()
					x[6] += 1
					connector7.cb_append_data_point(float(value), x[6])
				except queue.Empty:
					continue
			if not data_queue8.empty():
			# getting data from serial queue
				try:
					value = data_queue8.get_nowait()
					x[7] += 1
					connector8.cb_append_data_point(float(value), x[7])
				except queue.Empty:
					continue

	def data_generator_for4(data_queue1, connector1,
		    		   		data_queue2, connector2,
				       		data_queue3, connector3,
					   		data_queue4, connector4):
		x = [0, 0, 0, 0]
		while True:
			if not data_queue1.empty():
			# getting data from serial queue
				try:
					value = data_queue1.get_nowait()
					x[0] += 1
					connector1.cb_append_data_point(float(value), x[0])
				except queue.Empty:
					continue
			if not data_queue2.empty():
			# getting data from serial queue
				try:
					value = data_queue2.get_nowait()
					x[1] += 1
					connector2.cb_append_data_point(float(value), x[1])
				except queue.Empty:
					continue
			if not data_queue3.empty():
			# getting data from serial queue
				try:
					value = data_queue3.get_nowait()
					x[2] += 1
					connector3.cb_append_data_point(float(value), x[2])
				except queue.Empty:
					continue
			if not data_queue4.empty():
			# getting data from serial queue
				try:
					value = data_queue4.get_nowait()
					x[3] += 1
					connector4.cb_append_data_point(float(value), x[3])
				except queue.Empty:
					continue
			
	def save_data(self):
		if self.flag_save_data.value == False:
			filename_words = len(re.findall(r'\w+', self.filename_edit.text()))
			if filename_words == 0:
				self.filename_edit.setStyleSheet('border: 1px solid red')
				self.filename_edit.setText("default_output")
				self.filename_queue.put(self.filename_edit.text())
				self.flag_save_data.value = True
			else:
				self.filename_edit.setStyleSheet('')
				self.filename_queue.put(self.filename_edit.text())
				self.flag_save_data.value = True
			self.led_write.setPixmap(QPixmap("img/green_led.png"))
		else:
			self.filename_edit.setText("")
			self.filename_edit.setStyleSheet('')
			self.flag_save_data.value = False
			self.led_write.setPixmap(QPixmap("img/red_led.png"))

	def data_writer(flag_save_data, csvname, writer_data_queue, writer_kill):

		data_directory = "../../data/"

		# Creates "data" folder directory if it does not exists
		if not os.path.isdir(data_directory):
			os.mkdir(data_directory)
			time.sleep(0.1)

		while not writer_kill.value:
			if flag_save_data.value == True:
				# Input string from filename_edit
				input_name = csvname.get()
				input_name = input_name.replace(' ','_')
				# Date
				year = str(datetime.datetime.now().year)
				month = str(datetime.datetime.now().month)
				day = str(datetime.datetime.now().day)
				hour = str(datetime.datetime.now().hour)
				minute = str(datetime.datetime.now().minute)
				second = str(datetime.datetime.now().second)
				# Final filename
				filename = year + month + day + '_' + hour + minute + second + '_' + input_name
				filepath_name = data_directory + filename + '.csv'
				# Opening new file
				output_file = open(filepath_name,'w')
				time.sleep(0.1)  # waiting for the file to be opened
				
				while flag_save_data.value == True and not writer_kill.value:
					try:
						values = writer_data_queue.get(False)  # dont wait for new data 
						output_file.write(",".join(values) + "\n")
					except queue.Empty:
						continue
					except Exception as e:
						print(e)
						break

				output_file.close()
				print(filepath_name)
	
	def ss_timer(self): # button to start-stop streaming of serial data from feather
		if self.timer.isActive():
			self.timer.stop()
		else:	
			self.timer.start()	
	
	def send_msg(self,msg):
		self.serial_commands_queue.put(msg)

	def send_2msg(self, msg1, msg2):
		if DEBUG:
			print(f'Pre size: {self.serial_commands_queue.qsize()}')
		self.serial_commands_queue.put(msg1)
		if DEBUG:
			print(f'1 Command added: {msg1.encode("utf-8")} size: {self.serial_commands_queue.qsize()}')
		self.serial_commands_queue.put(msg2)
		if DEBUG:
			print(f'2 Command added: {msg2.encode("utf-8")} size: {self.serial_commands_queue.qsize()}')

	def setPressure(self, selector):
		if selector==1:
			tmp = self.press_spinbox.value()
		elif selector==2:
			tmp = self.pressFatigue_spinbox.value()

		if tmp<10:
			self.send_msg("300"+ str(tmp))
		else:
			self.send_msg("30"+ str(tmp))

	def setErrorThreshold(self):
		tmp = self.pressth_spinbox.value()*10
		if tmp<10:
			self.send_msg("400"+ str(tmp))
		elif tmp<100:
			self.send_msg("40"+ str(tmp))
	
	def setEMGbaseline(self):
		tmp = self.EMGbaseline_spinbox.value()
		if tmp<10:
			self.send_msg("600"+ str(tmp))
		elif tmp<100:
			self.send_msg("60"+ str(tmp))
		else:
			self.send_msg("6"+ str(tmp))

	def setCycles(self):
		tmp = self.cyclesFatigue_spinbox.value()
		tmp = tmp/10
		if tmp<10:
			self.send_msg("700"+ str(tmp))
		elif tmp<100:
			self.send_msg("70"+ str(tmp))
		else:
			self.send_msg("7"+ str(tmp))

		
	def update_data(self): # update plot and labels
		# loading all data from serial data queue
		while not self.update_gui_queue.empty():
			# getting data from serial queue
			try:
				gui_values = self.update_gui_queue.get_nowait()
				# update labels
				# --> EMG
				self.EMG_value.setText(gui_values[0])
				self.EMGbaseline_value.setText(gui_values[4])
				# --> IMU angles
				self.ELEV_value.setText(gui_values[18])
				# --> PRESSURE
				self.PRESS1_value.setText(gui_values[20])
				self.pref_value.setText(gui_values[21])
				self.prefFatigue_value.setText(gui_values[21])
				# update leds of pumps/valves status
				self.setLed(self.image1,gui_values[23])
				self.setLed(self.image2,gui_values[24])	
				self.setLed(self.image3,gui_values[25])
				# self.setLed(self.image4,gui_values[26])  # uncomment if valve 2 is installed in the robot
				self.setLed(self.led_IMUcalibration,gui_values[15] and gui_values[16])
				match gui_values[33]:
					case '0':
						self.control_mode_value.setText('idle')
						if not self.cyclesFatigue_spinbox.isEnabled():  # disabled if fatigue test is running -> reactivated when cycle is over
							self.abortFatigueTest()
					case '1':
						self.control_mode_value.setText('manual')
					case '2':
						self.control_mode_value.setText('pressure setpoint')
					case '3':
						self.control_mode_value.setText('fatigue test')
						self.cyclesFatigue_spinbox.setEnabled(False)
				# fatigue test variables
				self.currentCycle_value.setText(gui_values[27])
				self.neededCycles_value.setText(gui_values[28])
				# 5 seconds timer update
				time2print = math.floor((int(gui_values[34]) - self.timer_reset) * 1e-3)
				if DEBUG:
					print(f'Current value: {int(gui_values[34])} | Timer reset: {self.timer_reset} | Time in 5 s: {time2print} | five: {self.timer_five}')
				if time2print>=5 and self.timer_five:  # useful distinction if a timer that does two different counting times is required (now it does 5-5-5-5, can do 5-whatever-5-whatever)
					self.timer_reset = int(gui_values[34])
					self.timer_five = False
				elif time2print>=5 and not self.timer_five:
					self.timer_reset = int(gui_values[34])
					self.timer_five = True
				self.timer_value.setText(str(time2print+1))  # +1 to have a timer from 1 to 5 instead of from 0 to 4
				# number of kilos
				self.kilos_value.setText(gui_values[36])
				

			except queue.Empty:
				return

	def setLed(self, entity, value):
		if value=='1':
			entity.setPixmap(QPixmap("img/green_led.png"))
		else:
			entity.setPixmap(QPixmap("img/red_led.png"))

	def update_GUI(self): # time-based loop function 
		try:
			self.update_data()
		except Exception as e:
			print(e)
			pass

	def startFatigueTest(self):
		if self.cyclesFatigue_spinbox.value() == 0:
			self.cyclesFatigue_spinbox.setStyleSheet('border: 2px solid red')
		else:
			self.cyclesFatigue_spinbox.setStyleSheet('')
			self.send_msg("2002")

	def abortFatigueTest(self):
		self.cyclesFatigue_spinbox.setEnabled(True)
		if self.flag_save_data.value == True:  # stop saving data if doing so
			self.save_data()

	def closeEvent(self,event):
		if DEBUG:
			print('closing')
		self.send_msg("0001")  # reset arduino when GUI closes
		time.sleep(1)  # one second to allow communication with arduino
		self.writer_kill.value = True
		self.serial_kill.value = True
		self.update_plot_kill.value = True
		time.sleep(0.5)
		self.close()


def main():
	app = QApplication(sys.argv)
	w = Start_win() 
	w.show()
	sys.exit(app.exec_())

if __name__ == '__main__': 
	main()