#! /usr/bin/python
import sys
import Queue
import threading
import os
import getopt
import csv
import time

from PyQt4 import QtCore, QtGui
import numpy as np
from scipy import fftpack
import pyqtgraph as pg
import rpom_tools

class rpomPlot(pg.PlotWidget):
	def __init__(self, name):
		# Init parent
		super(rpomPlot,self).__init__(name)

class rpomUI(QtGui.QMainWindow):
	PLOTDEPTH = 1024*3
	QUEUEDEPTH = 500
	Ts = 1.0/1e3
	def __init__(self, parent=None):
		# Grab arguments
		try:
			opts, args = getopt.getopt(sys.argv[1:], "c:")
		except getopt.GetoptError, err:
			# print help information and exit:
			print str(err) # will print something like "option -a not recognized"
			#usage()
			sys.exit(2)
		if os.name == 'nt':
			self.portname = "\\\\.\\COM7"
		elif os.name == 'posix':
			self.portname = "/dev/ttyUSB1"
		for o, a in opts:
			if o == "-c":
				self.portname = a
			else:
				assert False, "unhandled option"
				
		# Init parents
		QtGui.QMainWindow.__init__(self)
		
		# Create the window
		self.create_main_frame()
		self.streaming_text = QtGui.QLabel('Monitor idle')
		self.port_text = QtGui.QLabel('Serial port closed')
		self.statusBar().addWidget(self.streaming_text, 1)
		self.statusBar().addWidget(self.port_text, 1)
		
		# Initiate a timer to control polling rate of the serial port queue
		self.timer = QtCore.QTimer()
		self.timer.setInterval(25)
		self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.timerEvent)	
		# Set up queue/FIFO for sync'ing serial data
		self.queue = Queue.Queue(50)		
		
		# Create serial stream
		self.stream = None
		self.streaming = False
		
		# Keyboard controls 
		# Toggle RPOM state
		self.stream_action = QtGui.QShortcut(self)
		self.stream_action.setKey("Space")
		self.connect(self.stream_action, QtCore.SIGNAL("activated()"), self.toggle_stream)

		# Initialize plot data
		self.t = np.arange(self.PLOTDEPTH)*self.Ts
		self.t_abs = np.arange(self.PLOTDEPTH, dtype=np.uint32)
		self.M = 4
		self.Mn =  np.int(np.floor(self.PLOTDEPTH/self.M)*self.M)
		self.f = np.linspace(0,2,self.Mn)/(2.0*self.Ts)
		self.ir = np.zeros(len(self.t), dtype='int32')
		self.red = np.zeros(len(self.t), dtype='int32')
		self.window = np.hanning(self.Mn)
	
	def timerEvent(self):	# Poll the data stream
		# Check for data
		self.getPacket()
		# Frequency analysis
		fir = np.abs(fftpack.fft(self.window*self.ir[-self.Mn:]))
		fred = np.abs(fftpack.fft(self.window*self.red[-self.Mn:]))
		
		# Redraw the raw plots
		self.curveIR.updateData(self.ir,x=self.t)
		self.curveRED.updateData(self.red,x=self.t)
		# Redraw the Fourier plots
		self.curveIRfreq.updateData(20.0*np.log10(fir[1:self.Mn]),x=self.f[1:])
		self.curveREDfreq.updateData(20.0*np.log10(fred[1:self.Mn]),x=self.f[1:])
	
	def getPacket(self):
		while not self.queue.empty():
			# Fetch raw data packet
			temp = self.queue.get(True)
			# Check packet is of the correct format
			if isinstance(temp, rpom_tools.RPOM_Packet):
				# TODO: Save data into HDF5 database				
				# Get packet length
				if isinstance(temp.ir,int): n = 1
				else: n = len(temp.ir)
				# Update the absolute time
				self.t_abs += n
				# Shift in new raw data
				self.ir[:-n] = self.ir[n:]
				self.ir[-n:] = temp.ir
				self.red[:-n] = self.red[n:]
				self.red[-n:] = temp.red
				# stdout a message in the queue starts getting arbitrarily deep
				if self.queue.qsize() >= 50:
					print "WARNING! Queue depth large: ", self.data_q.qsize()

	def start_stream(self):
		# Create a new thread
		self.stream = rpom_tools.rpomSerialThread(self.queue,self.portname,115200,1.0)
		# Start the sensor interface stream
		self.stream.start()
		# Start the system polling timer
		self.timer.start()
		# Set status message
		self.streaming_text.setText("Streaming live RPOM data")
		self.port_text.setText("COM port open")

	def stop_stream(self):
		# Stop the the timer and kill the force thread
		self.timer.stop()
		# Stop the sensor interface stream
		self.stream.join(timeout=5.0)
		self.stream = None
		# Set status message
		self.streaming_text.setText("RPOM sensor disabled")
		self.port_text.setText("COM port closed")

	def toggle_stream(self):
		if self.streaming:
			# Stop data stream
			self.stop_stream()
			# Set all flags false
			self.streaming = False
		else:
			# Start streaming data
			self.start_stream()
			self.streaming = True
		
	def create_main_frame(self):	
		# Create time series plot
		self.time_plot = pg.PlotWidget(name='TimePlot')
		# Create frequency domain plot
		self.freq_plot = pg.PlotWidget(name='FreqPlot')
		self.freq_h_plot = pg.PlotWidget(name='FreqHPlot')		
		
		# Create time based curves
		self.curveIR = self.time_plot.plot()
		self.curveIR.setPen("w")
		self.curveRED = self.time_plot.plot()
		self.curveRED.setPen("r")
		# Create frequency curves
		self.curveIRfreq = self.freq_plot.plot()
		self.curveIRfreq.setPen("w")
		self.curveREDfreq = self.freq_plot.plot()
		self.curveREDfreq.setPen("r")
		# Create envelope curves
		self.curveIRpeak = self.time_plot.plot()
		self.curveIRpeak.setPen("g")
		self.curveIRtrough = self.time_plot.plot()
		self.curveIRtrough.setPen("b")
		self.curveREDpeak = self.time_plot.plot()
		self.curveREDpeak.setPen("g")
		self.curveREDtrough = self.time_plot.plot()
		self.curveREDtrough.setPen("b")
		
		# Create the main frame and add plot and port widgets
		self.main_frame = QtGui.QWidget()
		main_layout = QtGui.QGridLayout()
		main_layout.addWidget(self.time_plot,0,0)
		main_layout.addWidget(self.freq_plot,1,0)
				
		# Instantiate the main window from the main frame layout
		self.main_frame.setLayout(main_layout)
		self.setCentralWidget(self.main_frame)
		self.resize(1000, 600)

	def display_about(self):
		QtGui.QMessageBox.about(self, "About BSNv3 Force Plate Monitor", __doc__)	

if __name__ == "__main__":
	app = QtGui.QApplication(sys.argv)
	ui = rpomUI()
	ui.show()	
	sys.exit(app.exec_())	
