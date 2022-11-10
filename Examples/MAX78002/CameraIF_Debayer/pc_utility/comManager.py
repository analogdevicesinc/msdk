import serial
import time
import os
import sys
import string

ser = serial.Serial()

def ByteToHex( ch ):
	try:
		ch = ord(ch)
	except:
		ch = 0

	return ch

def init(comPort, baudRate):
	#ser = serial.Serial('COM6', 115200, timeout=0)
	global ser
	ser.baudrate = baudRate
	ser.port = str(comPort)
	ser.timeout=3
	try:
		ser.open()
	except:
		None
	if ser.is_open:
		return 0
	else:
		return 1

magic_str = "*RDY*"
start_str = ""


PRTCL_STATE_FREE  = 0
PRTCL_STATE_1	  = 1,
PRTCL_STATE_2	  = 2,
PRTCL_STATE_3	  = 3,
PRTCL_STATE_4	  = 4,
PRTCL_STATE_FOUND = 5,

sync_word_state = PRTCL_STATE_FREE

def find_sync( ):
	global start_str
	global sync_word_state
	isFind = 0

	if True:
		for x in range(1, 1000):
			byt = ser.read(1)
			if sync_word_state == PRTCL_STATE_FREE:
				if (b'*' == byt):
					sync_word_state = PRTCL_STATE_1

			elif sync_word_state == PRTCL_STATE_1:
				if (b'S' == byt):
					sync_word_state = PRTCL_STATE_2
				elif (b'*' == byt):
					sync_word_state = PRTCL_STATE_1
				else:
					sync_word_state = PRTCL_STATE_FREE

			elif sync_word_state == PRTCL_STATE_2:
				if (b'T' == byt):
					sync_word_state = PRTCL_STATE_3
				elif (b'*' == byt):
					sync_word_state = PRTCL_STATE_1
				else:
					sync_word_state = PRTCL_STATE_FREE

			elif sync_word_state == PRTCL_STATE_3:
				if (b'R' == byt):
					sync_word_state = PRTCL_STATE_4
				elif (b'*' == byt):
					sync_word_state = PRTCL_STATE_1
				else:
					sync_word_state = PRTCL_STATE_FREE

			elif sync_word_state == PRTCL_STATE_4:
				if (b'*' == byt):
					sync_word_state = PRTCL_STATE_FREE
					isFind = 1
					break
				else:
					sync_word_state = PRTCL_STATE_FREE
	else:
		for x in range(1, 1000):
			byt = ser.read(1)

			if ((ByteToHex(byt) < 32) or  (ByteToHex(byt) > 127) ):
				byt = '-'

			try:
				ch = byt.decode("ascii")
			except :
				ch = str(byt)

			if (len(start_str) == 5):
				start_str = start_str[1:] + ch
			else:
				start_str = start_str[0:] + ch

			if start_str == magic_str:
				isFind = 1
				break
	return isFind

def read( expectedLen ):
	data=bytearray()
	data = ser.read(expectedLen)
	return data
