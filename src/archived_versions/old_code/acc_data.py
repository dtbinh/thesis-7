from timeit import Timer
import time
import binascii as b
import pprint as p #-------------------------- pretty print module
import serial
ser = serial.Serial(#------------- configure the serial connection
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout = 0
)
ser.open()
print ser.isOpen()# ----------------prints true if connection open
p.pprint(ser.getSettingsDict())
					#print the dictionary of serial settings

"""
The purpose of "testfunc()" is to send a 1 byte command and read in the a/d value that is returned by the msp430
"""
def testfunc():
#	time.sleep(0.5)
	ser.write(b.unhexlify('a1'))
	
	if ser.inWaiting() > 0:
		val = b.hexlify(ser.read(1))
		return val
	else:
		return 'q'

data = []

def itertest(num):
	global data
	[data.append(testfunc()) for i in range(num)]
	return 1

t = Timer()
print 'The execution time was', t.timeit(itertest(10))

print data

#-----  unhexlify --> http://docs.python.org/library/binascii.html



#print b.hexlify(ser.read(1))
