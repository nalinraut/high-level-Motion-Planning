from serial import *
from filters import *

class readArduino:
	def __init__(self, port = '/dev/ttyACM0', baud = 9600):
		self.ser = Serial(port,baud)
		self.expo_filter = ExponentialFilter()
		self.dead_filter = DeadbandFilter(1.0)

	def read(self):
		try:
			data = self.ser.readline().split()
			int_data = []
			
			for datum in data:
				int_data.append(int(datum))
			print int_data

			processed_data = self.dead_filter.process(int_data)
			processed_data = self.expo_filter.process(processed_data)

		except SerialException:
			print "Error reading from Serial Port"
		return processed_data



#	def read(self):
#
#		port = '/dev/ttyACM0'
#		baud = 9600
#	
#		processor = myDeadbandFilter()
#	
#		ser = Serial(port, baud)
#		while True:
#			try:
#				data = ser.readline().split()
#				#print data			
#				int_data = []
#				for datum in data:
#					int_data.append(int(datum))
#				processed_data = processor.process(int_data)
#	
#				#print "processed: %r" % processed_data
#	
#			except SerialException:
#				print "Error reading from Serial Port."	
#		return processed_data



if __name__ == "__main__":
	ra = readArduino()
	while True:
		print ra.read()




#	filename = "arduino_output.csv"
#	f = open(filename,"w")
#
#	port = '/dev/ttyACM0'
#	baud = 9600
#
#	processor = myDeadbandFilter()
#
#	ser = Serial(port, baud)
#	while True:
#		try:
#			data = ser.readline().split()
#			print data			
#			int_data = []
#			for datum in data:
#				int_data.append(int(datum))
#			processed_data = processor.process(int_data)		
#			
#			f.write(str(data))
#			f.write("\n")
#			f.write("integer: ")
#			f.write(str(int_data))
#			f.write("\n")
#			f.write("processed: ")
#			f.write(str(processed_data))
#			f.write("\n")
#
#			print "processed: %r" % processed_data
#
#		except SerialException:
#			print "Error reading from Serial Port."	
#	f.close()
