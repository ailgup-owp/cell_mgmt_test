"""
Cell Managment Board Test Script

2/20/19
C. Puglia

ex. python3 cell_mgmt.py -r H
"""



import serial
import visa
import sys
import time
from serial.tools import list_ports
from BKLoad import BKLoad
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
    
    
"""
This piece will need to be updated when a new Rev is released
"""    
TWELVE_CELL_REVS = ["F","H"]
FIFTEEN_CELL_REVS = ["I"]
DEFAULT_FILE_NAME = "C:\\Users\\Electrical  Test\\Dropbox (Open Water Power)\\Aluminum Research\\Design and Modeling\\Electrical\\Bench Testing\\Cell Managment Rev %s Bench Test.xlsx"


##############################################################
##############################################################
#####                                                    #####
#####               EDIT BELOW AT OWN RISK               #####
#####                                                    #####
##############################################################
##############################################################


""" ARGUMENT SECTION """
err = sys.stderr.write
parser = argparse.ArgumentParser()
parser.add_argument("-s","--skip", type=int, choices=[1, 2], help="skip parts of the test, 1=skip current meas, 2=skip current & voltage")
parser.add_argument("-sn","--serial", type=int, help="Board serial number, if not entered user will be asked for it")
parser.add_argument("-f","--file", type=str, help="File to save to, if none selected default is used")
parser.add_argument("-r","--rev", type=str, help="[REQUIRED] revision letter of PCB", required=True)

args = parser.parse_args()

serial_number=None

system("cls")

""" TEST VARIABLES """
PRIMARY=True
passing=True
pressure_test=None
led_test="Null"
stack_voltage=-1
measured_volts=[]
measured_current=[]
filename=None
try:
  filename = DEFAULT_FILE_NAME % args.rev[0].upper()
  x=open(filename)
  x.close()  
except:
  raise Exception("Cannot find file to write to, ensure revision correct")

if args.serial:
  serial_number=args.serial

def number_of_cells():
    if args.rev[0].upper() in TWELVE_CELL_REVS:
      return 12
    elif args.rev[0].upper() in FIFTEEN_CELL_REVS:
      return 15
    else:
      raise Exception("This program is not designed for this revision %s of the Cell Mgmt board" % args.rev[0].upper())
def cell_shorting(ser):
    #print("Cell Shorting: Resetting all FET's")
    numCells=number_of_cells()
    for i in range (1,numCells+1):
        ser.write(("sc+cellstat="+str(i)+",0\r\n").encode())
        ser.read(9999)
        time.sleep(0)
    #print("Cell Shorting: Reset complete")
    input("Switch LED on for shorting [ENTER]")
    for i in range (1,numCells+1):
        # if i>13:
        #   input("press enter to short")
        
        ser.write(("sc+cellstat="+str(i)+",1\r\n").encode())
        print("Cell Shorting: Shorting FET #",i)
        ser.read(9999)
        time.sleep(0)
    time.sleep(2)
    input("Cell Shorting: Press [ENTER] to begin reset")
    print("Cell Shorting: Resetting all FET's")
    for i in reversed(range (1,numCells+1)):
        ser.write(("sc+cellstat="+str(i)+",0\r\n").encode())
        ser.read(9999)
        time.sleep(0)
    
    print("Cell Shorting: Test Complete")

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

    lj.i2c(0x48, [0x00], SDAPinNum=14, SCLPinNum=15)
    lj.i2c(0x49, [0x00], SDAPinNum=14, SCLPinNum=15)

except:
    raise
    print("Unable to connect to Lab Jack, try a reconnect")
    exit(1)

#Turn OFF Relay
lj.setDIOState(16,0)

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
load=BKLoad()


#All HW good ready to test
serial_number=input("Serial number:")


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
    lj.setDIOState(16,1)
    
    pow_supply.write("VOLTage:LEVel 5.0\n")
    pow_supply.write("CURRent:LEVel 32\n")
    pow_supply.write("OUTPut ON\n")
    #Do current sweep
    
    #set remote on
    load.remote()
    #turn on
    load.on()
    #set cc mode
    load.setCC()
    current_steps=[0,1,5,10,20,30]
    
    for i in current_steps:
      print("Current Calibration: ",i,"A")
      load.set_current(i)
      time.sleep(2)
      measured_current.append(calc_avg_current(single_board,is_primary,5))
      if (i>1 and (abs((measured_current[-1]-(1000*i))/(1000*i))>.25)):
        print("FAILED!! Current sense is far off from calibration, measured:",measured_current[-1]/1000," where ",i," was expected")
        load.off()
        load.remote_off()
        pow_supply.write("OUTPut OFF\n")
        exit(1)
      print("Current Calibration: Measured current= ",measured_current[-1],"mA")
    #turn off &remove remote
    load.off()
    load.remote_off()
    pow_supply.write("OUTPut OFF\n")
    [current_slope,current_offset]=calc_slope_offset(current_steps,measured_current)
    print("Current Calibration: ",current_slope,"  ",current_offset)
    lj.setDIOState(16,0)
###Prompt removal of HV Connector


"""
LJ Voltage Check
"""
CELL_VOLTAGE = .2 #Volts
ACCEPTABLE_ERROR = .05 #%/100
NUMBER_OF_CELLS = number_of_cells()
if args.skip == 2:
    pass
else:
    cont=True
    lj.i2c(0x48, [0xFF], SDAPinNum=14, SCLPinNum=15)
    lj.i2c(0x49, [0xFF], SDAPinNum=14, SCLPinNum=15)
    while cont:
        measured_volts=[]
        #setting all Ticks to 200mV
        for i in range (0,8):
          if i==0:
            lj.writeRegister(5000, CELL_VOLTAGE)
          else:
            set_tick(lj,(i-1)*2,(0.4*i),(0.4*i+.2))
            #print((i-1)*2,(0.4*i),(0.4*i+.2))
        #Loops through all ADC chanels and looks "tries" number of times to see if the correct value is set one of those times
        #If none of those times are valid it writes the invalid value and keeps going
        # This is done to allow for easier debug if multiple channels are dead you see that
        for i in range(1,NUMBER_OF_CELLS+1):
            tries=0
            while (tries <4):
              single_board.write(("sc+csense="+str(1 if is_primary else 2)+","+str(i)+"\r\n").encode())
              val=int(single_board.read(9999).decode().split(" mV")[0])
              #If voltage is invalid
              if CELL_VOLTAGE*(1+ACCEPTABLE_ERROR) < val < CELL_VOLTAGE*(1-ACCEPTABLE_ERROR):
                time.sleep(.25)
                tries=tries+1
                if tries==4:
                    measured_volts.append(val)
              else:
                measured_volts.append(val)
                tries=5
                break
        #stack voltage
        single_board.write(("sc+cvstack="++","+str(i)+"\r\n").encode())
        val=int(single_board.read(9999).decode().split(str('b1: ' if is_primary else 'b2: '))[1].split(" mV")[0])
        if  NUMBER_OF_CELLS*CELL_VOLTAGE*(1-ACCEPTABLE_ERROR) < val < NUMBER_OF_CELLS*CELL_VOLTAGE*(1+ACCEPTABLE_ERROR):
            stack_voltage=val
        else:
            print("FAILED!!: Stack Voltage ",stack_voltage," not read correctly ")
            print(measured_volts)
            ask=input("Try again? [ENTER=yes]")
            if ask is "":
                pass
            else:
                passing=False
                break
        
        #checking the individual cell voltages
        if max(measured_volts) > CELL_VOLTAGE*(1+ACCEPTABLE_ERROR) or min(measured_volts) < CELL_VOLTAGE*(1-ACCEPTABLE_ERROR):
            passing=False
            if max(measured_volts) >CELL_VOLTAGE*(1+ACCEPTABLE_ERROR):
                offending_value = max(measured_volts)
            else:
                offending_value = min(measured_volts)
            print("FAILED!!: Cell Voltage ",offending_value," not read correctly on cell #",measured_volts.index(offending_value)+1)
            print(measured_volts)
            ask=input("Try again? [ENTER=yes]")
            if ask is "":
                pass
            else:
                passing=False
                break            
        else:
            print("Measured Voltages: ",measured_volts)
            cont=False
    lj.i2c(0x48, [0x00], SDAPinNum=14, SCLPinNum=15)
    lj.i2c(0x49, [0x00], SDAPinNum=14, SCLPinNum=15)



# DO LED shorting test
cell_shorting(single_board)
val=input("Did LED's illuminate in order? [ENTER=yes]")
if val == '':
  pass
  led_test="GOOD"
else:
  passing=False
  led_test="FAIL"
  print("FAILED!!: Cell Shorting Falied")
  exit(1)
print("Turn off LED switch")
print("\nPASS")
ask=input("Save to file? [ENTER=yes]")
if ask == '':
    x=Excel(filename)
    x.writeToFile([serial_number,measured_current,measured_volts.append(stack_voltage),led_test,pressure_test],is_primary)
    x.saveFile()
else:
    pass
pow_supply.close()
exit(1)