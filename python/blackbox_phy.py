import ctypes
import serial

class BlackBox_Packet(ctypes.Structure):
	_fields_ = [
		('source',ctypes.c_uint8),
		('type',ctypes.c_uint8),
		('length',ctypes.c_uint8),
		('checksum',ctypes.c_uint8),
		('payload',ctypes.c_uint8*188)
	]

class BlackBoxStreamParser(object):
	def __init__(self):
		# Set up stream variables
		self.stream = None
		self.stream_open = False
		self.remainder = ""
	
	
	def read(self):
		if (self.stream_open == False):
			print "Stream not open - read failed."
			return 1
		
		# First check for a header
		if (len(self.remainder)<4):
			_inarray = self.stream.read(4-len(self.remainder))
			# Test if there are enough bytes left in the file
			if ((len(_inarray)+len(self.remainder))< 4):
				#print "Not enough for header:", len(_inarray), len(self.remainder)
				return 2
			if len(_inarray)==0:
				#print "No bytes available to read."
				return 2
			# Concatenate new bytes into array
			_inarray = self.remainder + _inarray
		else:
			_inarray = self.remainder


		# Convert the characters into bytes
		raw_packet = []
		for x in _inarray: raw_packet.append(ord(x))
		
		# Look for a packet header
		test_packet = BlackBox_Packet()
		
		test_packet.source		= raw_packet[0]
		test_packet.type		= raw_packet[1]
		test_packet.length		= raw_packet[2]
		test_packet.checksum	= raw_packet[3]

		# Test source ID
		if	(test_packet.source >= 16):
			print "Invalid source ID: ", test_packet.source
			self.remainder = _inarray[1:]
			return -1
			
		# Test packet type
		if	(test_packet.type >= 32):
			self.remainder = _inarray[1:]
			print "Invalid packet type", test_packet.type
			self.remainder = _inarray[1:]
			return -2

		# Test length
		if	(test_packet.length < 6) | \
			(test_packet.length > 192):
			self.remainder = _inarray[1:]
			print "Invalid packet length", test_packet.length
			return -3
			
		# Header is valid so update remainder array and fetch the payload
		self.remainder = _inarray
		
		_inarray = self.stream.read(test_packet.length-len(self.remainder))
		# Test if there are enough bytes left in the file
		if (len(_inarray) < test_packet.length-len(self.remainder)):
			#print "Not enough for payload:", len(_inarray), len(self.remainder)			
			return 2
		if len(_inarray)==0:
			#print "No bytes available to read."
			return 2
		# Concatenate new bytes into array
		_inarray = self.remainder + _inarray
		# Convert the characters into bytes
		raw_packet = []
		for x in _inarray: raw_packet.append(ord(x))

		# Extract data
		for i in range(test_packet.length-4):
			test_packet.payload[i] = raw_packet[i+4]

		# Test checksum
		if (test_packet.checksum != 0):
			w_checksum = test_packet.source
			w_checksum += test_packet.type
			w_checksum += test_packet.length
			for x in test_packet.payload: w_checksum += x
			w_checksum = 0x00FF&( ((w_checksum & 0xFF00)>>8) + (w_checksum & 0x00FF) )
			if (w_checksum != test_packet.checksum):
				print "BAD checksum: ", hex(w_checksum), " != ", hex(test_packet.checksum)
				self.remainder = _inarray[1:]
				return -4
					
		# If it has made it this far then the packet should be okay 
		self.remainder = _inarray[test_packet.length:]
		return test_packet

class BlackBoxFileStream(BlackBoxStreamParser):
	def __init__(self, file_name = 'DATA0.BIN'):	
		super(BlackBoxFileStream,self).__init__()
		# Set up file definitions
		self.file_name = file_name
		
	def open(self):
		# Try to open the file
		if (self.stream_open == True):
			print "File already open."
			return 1
		self.stream = open(self.file_name, 'rb')
		self.stream_open = True
		print "File opened."
		# Fetch the file header
		_inarray = self.stream.read(16)		
		# Test for a file header
		if (_inarray[0:3] != 'BSN'):
			print 'Invalid file header'
			self.remainder = _inarray
			return
		# Parse and print file ID
		print 'File version: {0}.{1}'.format(ord(_inarray[3]), ord(_inarray[4]))
		print 'Board type: {0}'.format(ord(_inarray[5]))
		print 'Hardware version: {0}.{1}'.format(ord(_inarray[6]), ord(_inarray[7]))
		print 'Firmware version: {0}.{1}'.format(ord(_inarray[8]), ord(_inarray[9]))
		print 'Board ID: {0:0>2x}:{1:0>2x}:{2:0>2x}:{3:0>2x}:{4:0>2x}:{5:0>2x}'.format( ord(_inarray[10]), ord(_inarray[11]), ord(_inarray[12]), ord(_inarray[13]), ord(_inarray[14]), ord(_inarray[15]))

	def close(self):
		if (self.stream_open == False):
			print "File not open."
			return 1
		self.stream.close()
		self.stream_open = False
		print "File closed."

class BlackBoxSerialStream(BlackBoxStreamParser):
	def __init__(	self, 
				port_name = '/dev/ttyACM0',
				port_baud = 115200,
				port_timeout=1.0):
		super(BlackBoxSerialStream,self).__init__()
		
		# Set up serial port definitions
		self.stream = serial.Serial()
		self.stream.port = port_name
		self.stream.baudrate = port_baud
		self.stream.timeout = port_timeout
		
	def open(self):
		self.stream.open()
		if not self.stream.isOpen():
			print "Damn. Wouldn't open"
			pass
		self.stream.flush()
		self.stream_open = True

	def close(self):
		self.stream.close()
		self.stream.flush()
		self.stream_open = False

if __name__ == "__main__":
	# Example of how to stream blackbox data from file
	bb_stream = BlackBoxSerialStream('\\\\.\\COM7',115200,1.0)
	bb_stream.open()
	for x in range(100):
		temp = bb_stream.read()
		# Check for error
		if isinstance(temp,int):
			if temp == 1:
				print "File not actually open yet - quitting"
				break
			elif temp == 2:
				print "Closing file as nothing left to read."
				bb_stream.close()
				break
			else: print("Error: %d"%temp)
		# Else process the packet
		elif isinstance(temp,BlackBox_Packet):
			print("Source ID: %d, packet format: %d, packet length: %d" % (temp.source, temp.type, temp.length))
			if temp.type == 0:
				print("RPOM samples: %d,\t %d" % ((temp.payload[0]<<8) + temp.payload[1],(temp.payload[2]<<8) + temp.payload[3]))
	bb_stream.close()
