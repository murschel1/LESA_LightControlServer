
'''
Author: Arunas Tuzikas
Description: Do implementation of the light system, set and read parameters.
Date: 9/15/2016
'''

import socket
import threading
import sys
import time
import smbus
import RPi.GPIO as GPIO

bus = smbus.SMBus(1)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup( 9, GPIO.OUT, initial=GPIO.LOW) # UV off
GPIO.setup(26, GPIO.OUT, initial=GPIO.LOW) # Deep Blue off
GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW) # Blue off
GPIO.setup(13, GPIO.OUT, initial=GPIO.LOW) # Green of
GPIO.setup(14, GPIO.OUT, initial=GPIO.LOW) # Red off
GPIO.setup(15, GPIO.OUT, initial=GPIO.LOW) # Far Red off

GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW) # Fan

#I2C addresses
addr_ADC = 0x37 # Temperature sensor
addr_DAC = 0x14 # Anallog signal to drivers



#Global constant used by the server
TCP_IP =  '192.168.0.249'#'local host'
TCP_SERVER_PORT = 50000
BUFFER_SIZE = 1000

def InitTempSens():
  try:
    while bus.read_byte_data(addr_ADC, 0x0C) & 0x02:	# Wait for device to get ready
      print ("ADC is not ready")
    bus.write_byte_data(addr_ADC, 0x0B, 0x00)
    bus.write_byte_data(addr_ADC, 0x08, 0x00)
    return True
  except:
   print ("ADC Error:", sys.exc_info()[1])
   return -1
 
def ReadTempSens(sensor):
  try:
    channel_base = 0x20
    bus.write_byte_data(addr_ADC, 0x09, 0x01) 	#Start a conversion
    while bus.read_byte_data(addr_ADC, 0x0C) & 0x01: 
      pass
    return bus.read_byte_data(addr_ADC, channel_base + int(sensor))
  except:
    print ("ADC Error:", sys.exc_info()[1])
    return -1

def Fan(status):
  if status == 1:
    GPIO.output(21, 1)
  else:
    GPIO.output(21, 0)
  
def AllDis():
  GPIO.output( 9, 0) #UV
  GPIO.output(26, 0) #DB
  GPIO.output(12, 0) #BL
  GPIO.output(13, 0) #GR
  GPIO.output(14, 0) #RE
  GPIO.output(15, 0) #IR

def AllEna():
  GPIO.output( 9, 1) #UV
  GPIO.output(26, 1) #DB
  GPIO.output(12, 1) #BL
  GPIO.output(13, 1) #GR
  GPIO.output(14, 1) #RE
  GPIO.output(15, 1) #IR


def InitDAC():
  try:
    bus.write_i2c_block_data(addr_DAC, 0b00100100, [0, 0])	#external reference
    return True
  except:
    print ("ADC Error:", sys.exc_info()[1])
    return -1

def SetDAC(UV, DB, BL, GR, RE, IR):
  data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  data[ 0] = (int(40.95*(100 - UV)) >> 4)
  data[ 1] = (int(40.95*(100 - UV)) << 4) & 0x00F0
  data[ 2] = (int(40.95*(100 - DB)) >> 4)
  data[ 3] = (int(40.95*(100 - DB)) << 4) & 0x00F0
  data[ 4] = (int(40.95*(100 - BL)) >> 4)
  data[ 5] = (int(40.95*(100 - BL)) << 4) & 0x00F0
  data[ 6] = (int(40.95*(100 - GR)) >> 4)
  data[ 7] = (int(40.95*(100 - GR)) << 4) & 0x00F0
  data[ 8] = (int(40.95*(100 - RE)) >> 4)
  data[ 9] = (int(40.95*(100 - RE)) << 4) & 0x00F0
  data[10] = (int(40.95*(100 - IR)) >> 4)
  data[11] = (int(40.95*(100 - IR)) << 4) & 0x00F0
  
  print data
  try:
    bus.write_i2c_block_data(addr_DAC, 0b10100000, [data[ 0], data[ 1]])
    bus.write_i2c_block_data(addr_DAC, 0b10100001, [data[ 2], data[ 3]])
    bus.write_i2c_block_data(addr_DAC, 0b10100010, [data[ 4], data[ 5]])
    bus.write_i2c_block_data(addr_DAC, 0b10100011, [data[ 6], data[ 7]])
    bus.write_i2c_block_data(addr_DAC, 0b10100100, [data[ 8], data[ 9]])
    bus.write_i2c_block_data(addr_DAC, 0b10100101, [data[10], data[11]])
    return True
  except:
    print ("ADC Error:", sys.exc_info()[1])
    return -1

def main():

  InitTempSens()
  InitDAC()
  Fan(0)
  AllDis()
  UV = 0
  DB = 0
  BL = 0
  GR = 0
  RE = 0
  IR = 0
  
  print ("Running Light Controll Server")
  while 1:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      s.bind((TCP_IP, TCP_SERVER_PORT))
    except socket.error, msg:
      print ("Bind failed. Error code: " + str(msg[0]) + " Message: " + msg[1])
      sys.exit()
    print ("Socket bind complete")

    s.listen(1)
    while 1:
      print ("Waiting for client")
      (client, address) = s.accept()
      print ("Accepted client")
      while 1:
        # If the client breaks connection, start accepting more clients
        try:
          data = client.recv(BUFFER_SIZE)
        # If the client terminates the connection, go back and wait for another client
        except:
          break
        if not data:
          break
        data = data.strip().split()
 #       print data[0], data[1] #This data should be using to set DAC and PWM
        if data[0] == 'FAN':
          Fan(int(data[1]))
        if data[0] == 'TEM':
          print ("Temperature of Sensor " + str(data[1]) + " is " + str(ReadTempSens(data[1])) + " in Celcius")
          try: 
            client.sendall(str(ReadTempSens(data[1]))+"\n")
          except:
            print ("Failed to send massage")
        if data[0] == "UVX":
          try:
            UV = float(data[1])
            SetDAC(UV, DB, BL, GR, RE, IR)
          except:
            UV = 0
        elif data[0] == "DBL":
          try:
            DB = float(data[1])
            SetDAC(UV, DB, BL, GR, RE, IR)
          except:
            DB = 0
        elif data[0] == "BLU":
          try:
            BL = float(data[1])
            SetDAC(UV, DB, BL, GR, RE, IR)
          except:
            BL = 0
        elif data[0] == "GRE":
          try:
            GR = float(data[1])
            SetDAC(UV, DB, BL, GR, RE, IR)
          except:
            GR = 0
        elif data[0] == "RED":
          try:
            RE = float(data[1])
            SetDAC(UV, DB, BL, GR, RE, IR)
          except:
            RE = 0
        elif data[0] == "IRX":
          try:
            IR = float(data[1])
            SetDAC(UV, DB, BL, GR, RE, IR)
          except:
            IR = 0
        elif data[0] == "ENA":
          AllEna()
        elif data[0] == "DIS":
          AllDis()
        



if __name__ == '__main__':
	main()
