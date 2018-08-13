"""
Cell Managment Board Test Script

8/7/2018
C. Puglia

ex. python3 cell_mgmt.py
"""
import serial
import visa
import sys
import time
from serial.tools import list_ports
import numpy as np
from os import system
from numpy import linalg
import struct,sys, dcload, u6
import argparse
from Excel import Excel
try:
    from win32com.client import Dispatch
except:
    pass
err = sys.stderr.write
parser = argparse.ArgumentParser()
parser.add_argument("-s","--skip", type=int, choices=[1, 2], help="skip parts of the test, 1=skip current meas, 2=skip current & voltage")
parser.add_argument("-sn","--serial", type=int, help="Board serial number, if not entered user will be asked for it")

parser.add_argument("-f","--file", type=str, help="File to save to, if none selected default is used")

args = parser.parse_args()
system("cls")
PRIMARY=True
passing=True

pressure_test=None
led_test="Null"
measured_volts=[]
measured_current=[]

DEFAULT_EXCEL_FILE_PATH="C:\\Users\\Electrical  Test\\Dropbox (Open Water Power)\\Aluminum Research\\Design and Modeling\\Electrical\\Bench Testing\\Cell Managment Rev D Bench Test.xlsx"
if args.file:
    DEFAULT_EXCEL_FILE_PATH=args.file

if not args.serial:
  serial_number=input("Serial number:")
else:
  serial_number=args.serial
    
def cell_shorting(ser):
	print("Cell Shorting: Resetting all FET's")
	for i in range (1,16):
		ser.write(("sc+cellstat="+str(i)+",0\r\n").encode())
		ser.read(9999)
		time.sleep(0)
	print("Cell Shorting: Reset complete")
	for i in range (1,16):
		# if i>13:
		#	input("press enter to short")
		
		ser.write(("sc+cellstat="+str(i)+",1\r\n").encode())
		print("Cell Shorting: Shorting FET #",i)
		ser.read(9999)
		time.sleep(0)
	time.sleep(2)
	input("Cell Shorting: Press enter to begin reset")
	print("Cell Shorting: Resetting all FET's")
	for i in reversed(range (1,16)):
		ser.write(("sc+cellstat="+str(i)+",0\r\n").encode())
		ser.read(9999)
		time.sleep(0)
	
	print("Cell Shorting: Test Complete")
def set_current(load,current=0):
    if current==0:
      load.write(bytearray([0xaa, 0, 0x2a, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xd4]))
    elif current==1:
      load.write(bytearray([0xaa, 0, 0x2a, 0x10,0x27,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xb]))
    elif current==5:
      load.write(bytearray([0xaa, 0, 0x2a, 0x50,0xC3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xE7]))
    elif current==10:
      load.write(bytearray([0xaa, 0, 0x2a, 0xA0,0x86,0x01,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xfb]))
    elif current==20:
      load.write(bytearray([0xaa, 0, 0x2a, 0x40,0x0D,0x03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x24]))
    elif current==30:
      load.write(bytearray([0xaa, 0, 0x2a, 0xE0,0x93,0x04,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x4b]))
    else:
      raise Exception("Invalid current")
def calc_avg_current(board,primary,counts=10):
    avg=[]
    for i in range(counts):
      board.write(("sc+ciout\r\n").encode())
      time.sleep(1)
      val=board.read(9999).decode().split("\r")[:2]
      for v in range(len(val)):
        val[v]=int(val[v].split(" ")[1])
      if primary:
        avg.append(val[0])
        #if len(avg)>0 and (val[0]-avg[-1]:
        
      else:
        avg.append(val[1])
        print(val[1])
    return sum(avg)/counts
def calc_slope_offset(set_current,meas_current):
    x = np.array(meas_current)
    y = np.array(set_current)
    A = np.vstack([x, np.ones(len(x))]).T
    m, c = linalg.lstsq(A, y, rcond=None)[0]
    return [m*1000,c*1000]
    
def toDouble(buffer):
    right, left = struct.unpack("<Ii", struct.pack("B" * 8, *buffer[0:8]))
    return (float(left) + float(right)/(2**32))
def set_tick_hi_z(lj,number):
    lj.i2c(0x12,[0x4f,0,0],SDAPinNum=number+1, SCLPinNum=number)
def set_tick(lj,number,voltA,voltB):
    sclPin = number
    sdaPin = sclPin + 1
    data = lj.i2c(0x50, [64], NumI2CBytesToReceive=36, SDAPinNum=sdaPin, SCLPinNum=sclPin)
    response = data['I2CBytes']
    aSlope = toDouble(response[0:8])
    aOffset = toDouble(response[8:16])
    bSlope = toDouble(response[16:24])
    bOffset = toDouble(response[24:32])
    try:
        lj.i2c(0x12, [48, int(((voltA*aSlope)+aOffset)/256), int(((voltA*aSlope)+aOffset) % 256)], SDAPinNum=sdaPin, SCLPinNum=sclPin)
        lj.i2c(0x12, [49, int(((voltB*bSlope)+bOffset)/256), int(((voltB*bSlope)+bOffset) % 256)], SDAPinNum=sdaPin, SCLPinNum=sclPin)
    except:
        raise Exception("Error Communicating with tick")
    
#Labjack Setup
try:
    lj=u6.U6()
except:
    print("Unable to connect to Lab Jack, try a reconnect")
    exit(1)
    #Turn ON Relay

#SingleBoard Setup
try:
    single_board = serial.Serial(list(list_ports.grep("0403:6001"))[0][0], 115200, timeout=.25)
except:
    print("Unable to connect to Single Board, try a reconnect")
    exit(1)
    
#Power Supply Setup
try:
    rm=visa.ResourceManager()
    pow_supply = rm.open_resource('USB0::0x0957::0xA107::US18C3185P::INSTR')
except:
    print("Unable to connect to Keysight N5700, try a reconnect")
    exit(1)
pow_supply.write("OUTPut OFF\n")

#BKLoad Setup
try:
    load=serial.Serial(list(list_ports.grep("067B:2303"))[0][0], 4800, timeout=.25)
except:
    print("Unable to connect to BKLoad, try a reconnect")
    exit(1)

#Turn off telemetry
single_board.write(("sc+telem=0\r").encode())
single_board.write(("sc+lgcfg=5,5\r").encode())
single_board.read(9999)

#Determine if primary or secondary
press=[]
for i in range(3):
    single_board.write(("sc+cpressure\r").encode())
    press.append(float((single_board.read(9999).decode()).split(" mbar")[0]))
    time.sleep(1)

if press[0] > 900 and press[0]<1200:
  if press[0]==press[1] and press[1]==press[2]:
      print ("Pressure not updating, try reconnecting single board")
      exit(1)
  else:
    is_primary=True
    pressure_test="GOOD"
    print("Primary: ",is_primary)
else:
  is_primary=False
  print("Primary: ",is_primary)

  #TODO add cmd line arg for skipping parts of the test for retesting
  #add csv/excell logging
  
  
"""
BK LOAD Current Cal
"""
if args.skip == 1 or args.skip == 2: 
    pass
else:
    pow_supply.write("VOLTage:LEVel 7.5\n")
    pow_supply.write("CURRent:LEVel 32\n")
    pow_supply.write("OUTPut ON\n")
    #Do current sweep
    
    #set remote on
    load.write(bytearray([0xaa, 0, 0x20, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
    #turn on
    load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
    #set cc mode
    load.write(bytearray([0xaa, 0, 0x28, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xd2]))
    current_steps=[0,1,5,10,20,30]
    
    for i in current_steps:
      print("Current Calibration: ",i,"A")
      set_current(load,i)
      time.sleep(2)
      measured_current.append(calc_avg_current(single_board,is_primary,5))
      if (i>1 and (abs((measured_current[-1]-(1000*i))/(1000*i))>.15)):
        print("FAILED!! Current sense is far off from calibration, measured:",measured_current[-1]/1000," where ",i," was expected")
        load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
        load.write(bytearray([0xaa, 0, 0x20, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xca]))
        pow_supply.write("OUTPut OFF\n")
        exit(1)
      print("Current Calibration: Measured current= ",measured_current[-1],"mA")
    #turn off &remove remote
    load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
    load.write(bytearray([0xaa, 0, 0x20, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xca]))
    pow_supply.write("OUTPut OFF\n")
    [current_slope,current_offset]=calc_slope_offset(current_steps,measured_current)
    print("Current Calibration: ",current_slope,"  ",current_offset)

###Prompt removal of HV Connector
input("Disconnect HV Connection [ENTER]")

"""
LJ Voltage Check
"""
if args.skip == 2:
    pass
else:
    cont=True
    input("Connect Lab Jack Connection [ENTER]")
    while cont:
        measured_volts=[]
        #setting all Ticks to 200mV
        for i in range (0,8):
          if i==0:
            lj.writeRegister(5000, .2)
          else:
            set_tick(lj,(i-1)*2,(0.4*i),(0.4*i+.2))
            #print((i-1)*2,(0.4*i),(0.4*i+.2))
        #TODO read back all ticks to verify they are set
        
        for i in range(1,16):
            tries=0
            while (tries <4):
              single_board.write(("sc+csense="+str(1 if is_primary else 2)+","+str(i)+"\r\n").encode())
              val=int(single_board.read(9999).decode().split(" mV")[0])
              if val >210 or val<190:
                time.sleep(.25)
                tries=tries+1
              else:
                measured_volts.append(val)
                tries=5
                break
          
        if max(measured_volts) >210 or min(measured_volts) <190:
            passing=False
            if max(measured_volts) >210:
                offending_value = max(measured_volts)
            else:
                offending_value = min(measured_volts)
            print("FAILED!!: Cell Voltage ",offending_value," not read correctly on cell #",measured_volts.find(offending_value)+1)
            print(measured_volts)
            ask=input("Try again? (y/n)")
            if ask is "n":
                exit(1)
            else:
              break    
        for i in range (0,8):
          if i==0:
            lj.writeRegister(5000, 0)
          else:
            set_tick(lj,(i-1)*2,(0),(0))
        print("Measured Voltages: ",measured_volts)
        cont=False

input("\nDisconnect Lab Jack connection and switch LED on for shorting [ENTER]")


# DO LED shorting test
cell_shorting(single_board)
val=input("Did LED's illuminate in order? (y/n)")
if val == 'y' or val == 'Y':
  pass
  led_test="GOOD"
else:
  passing=False
  led_test="FAIL"
  print("FAILED!!: Cell Shorting Falied")
  exit(1)
print("Turn off LED switch")
print("\n\nPASS")
ask=input("Save to file? (y/n)")
if ask == 'y' or ask == 'Y':
    x=Excel(DEFAULT_EXCEL_FILE_PATH)
    x.writeToFile([serial_number,measured_current,measured_volts,led_test,pressure_test],is_primary)
    x.saveFile()
else:
    pass
rm.close()
exit(1)