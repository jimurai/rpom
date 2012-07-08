import threading
import blackbox_phy

class RPOM_Packet():
	def __init__(self):
		self.ir = []
		self.red = []

class rpomSerialThread(threading.Thread):
	def __init__(	self, 
					data_queue,
					port_name = '/dev/ttyACM0',
					port_baud = 115200,
					port_timeout=1.0):
		# Init parent
		super(rpomSerialThread,self).__init__()
		# Init serial port
		self.serial_stream = blackbox_phy.BlackBoxSerialStream(port_name,port_baud,port_timeout)
		# Get reference to queue
		self.queue = data_queue		
		# Set up threading
		self.enabled = threading.Event()
		
	def run(self):
		# Open the serial port
		self.serial_stream.open()
		# Enable continuous sampling of serial port
		self.enabled.set()
		
		# If the RPOM needs any START commands put them here
		
		# Start collecting data and put it in a queue
		_index = 0
		_working_packet = RPOM_Packet()
		while self.enabled.isSet():
			if self.queue.full():
				# Keep an eye of the serial port buffer while waiting
				temp = self.serial_stream.stream.inWaiting()
				if temp > 512:
					print "Serial buffer deeper than 512 characters! Quiting..."
					# Cause the run() loop to complete and stop the thread
					self.enabled.clear()		
					self.serial_stream.close()
					raise Exception('Buffer overflow prevention')
			else:
				temp = self.serial_stream.read()
				# Check for error
				if isinstance(temp,int):
					if temp == 1:
						print "Stream not actually open yet - quitting"
						break
				# Else process the packet
				elif isinstance(temp,blackbox_phy.BlackBox_Packet):
					if temp.type == 0:
						_working_packet.red.append((temp.payload[0]<<8) + temp.payload[1])
						_working_packet.ir.append((temp.payload[2]<<8) + temp.payload[3])
						_index += 1
						if _index == 8:
							self.queue.put(_working_packet)
							_working_packet = RPOM_Packet()
							_index = 0
			
	def join(self, timeout=None):
		# If the RPOM needs any STOP commands put them here
		# Cause the run() loop to complete and stop the thread
		self.enabled.clear
		self.serial_stream.close()
		threading.Thread.join(self, timeout)

		# Sample application to print ADC readings to console
import time
import Queue
if __name__ == "__main__":
	# Set up queue/FIFO for sync'ing serial data
	data_q = Queue.Queue(50)	
	# Instantiate the force plate thread
	stream = rpomSerialThread(data_q,"\\\\.\\COM7",115200,1.0)	
	# Start the thread (which opens the port and starts parsing data)
	stream.start()
	
	# Read a few samples and print the values to the console
	sample_count = 0
	while sample_count < 1000:
		while not stream.queue.empty():
			data = stream.queue.get(True)
			if isinstance(data, RPOM_Packet):
				for x, y in zip(data.ir,data.red):
					print("Count: %d, IR: %d, Red: %d" % (sample_count, x, y))
					sample_count += 1
			else:
				print("Don\'t recognise that packet")
	# Kill the thread
	stream.join(timeout=1.0)	# Stop the thread
	stream=None