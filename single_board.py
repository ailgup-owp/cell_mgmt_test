"""
Single Board Test Script

11/1/2018
C. Puglia
OPNCS
ex. python3 cell_mgmt.py
"""
import serial
import visa
import sys
import time
import math
from serial.tools import list_ports
import numpy as np
from os import system
from numpy import linalg
import struct,sys, dcload, u6
import argparse
from Excel import Excel
from U1282A import U1282A
import statistics
# TODO add inpulse test


class Var:
  def __init__(self,name="",max=math.inf,min=-math.inf,type='bool'):
    self.name=name
    self.max=max
    self.min=min
    self.type=type
    self.passing=False
    self.value=None
  def isBool(self):
    return self.type=='bool'
    
  def isInt(self):
    return self.type=='int'
    
  def isFloat(self):
    return self.type=='float'
    
  def isStr(self):
    return self.type=='str'
    
  def isList(self):
    return self.type=='list'
    
  def setValue(self,val):
    self.value=val
    self.boundsCheck()
    if self.passing:
        print("[PASS] ",self.name," : ",val)
    else:
        print("[FAIL] ",self.name," : ",val)
        q=input("Press (p) to bench pass, otherwise test will end as FAIL>>")
        if q=='p':
            pass
        else:
            exit()
            #in the future give option to save and turn off PSU
  def getValue(self):
    return self.value
    
  def boundsCheck(self):
    if (self.isFloat() or self.isInt()):
      self.passing = (self.value >= self.min) and (self.value <= self.max)
    elif self.isList(): 
      if type(self.min)==list and type(self.max)==list:
        if len(self.max) == len(self.value) and len(self.min) == len(self.value):
          for i in range(len(self.value)):
            if self.value[i]>self.max[i] or self.value[i]<self.min[i]:
                self.passing = False
                return
          self.passing = True
          return
        else:
          raise Exception("Lengths inconsistant")
          self.passing=False
          return 
      else:
        self.passing = (min(self.value) >= self.min) and (max(self.value) <= self.max)
    elif self.isBool():
      self.passing = self.value
    elif self.isStr():
      self.passing = len(self.value) > 0
      
  def bounds_check(self):
    try:
        for x in mylist:
            pass
    except TypeError:
        if self.value <= self.max or self.value >= self.min:
            return True
      
class Tests:
    def __init__(self):
        self.sn=Var("SN",min=0,type='int')
        self.uid=Var("UID",type='str')
        
        self.input_32v_off=Var("Input 32V off",max=.1,min=-.1,type='list')
        self.input_32v_0a=Var("Input 32V 0A",max=.1,min=-.1,type='list')
        self.input_32v_0a5=Var("Input 32V 0.5A",max=1.5,min=.5,type='list')
        self.input_32v_1a=Var("Input 32V 1A",max=3,min=1.2,type='list')
        self.input_32v_1a5=Var("Input 32V 1.5A",max=4,min=1.5,type='list')
        self.input_32v_2a=Var("Input 32V 2A",max=5.5,min=2,type='list')
        
        self.output_32v_off=Var("Output 32V off",max=.1,min=-.1,type='list')
        self.output_32v_1a=Var("Output 32V 1A",max=2,min=0,type='list')
        self.output_32v_2a=Var("Output 32V 2A",max=3,min=1,type='list')
        self.output_32v_3a=Var("Output 32V 3A",max=4,min=2,type='list')
        self.output_32v_4a=Var("Output 32V 4A",max=5,min=3,type='list')
        self.output_32v_5a=Var("Output 32V 5A",max=6,min=4,type='list')
        self.output_32v_6a=Var("Output 32V 6A",max=7,min=5,type='list')
        
        self.input_cal=Var("Boost input calibration",max=[1.5,100],min=[.7,-100],type='list')
        self.output_cal=Var("Boost output calibration",max=[1.5,100],min=[.7,-100],type='list')
        
        self.output_3a_spot_check = Var("3A Spot Check",max=3.2,min=2.8,type='float')
        
        self.boost_regulation_2a=Var("Input Voltage Regulation, 2A",max=[10,7,2.2,32,7.1,8.5],min=[6,6.5,1.8,15,6.9,5.5],type='list')
        self.boost_regulation_3a=Var("Input Voltage Regulation, 3A",max=[10,7,3.2,32,7.1,8.5],min=[6.1,6.5,2.8,7,6.9,5.5],type='list')
        self.boost_regulation_4a=Var("Input Voltage Regulation, 4A",max=[610,7,4.3,15,7.1,8.5],min=[6.2,6.5,3.8,7,6.9,5.5],type='list')
        
        self.reg_12v=Var("12V Rail",max=12.2,min=11.8,type='float')
        self.reg_6v=Var("6V Rail",max=6.1,min=5.88,type='float')
        self.reg_5v5=Var("5.5V Rail",max=5.6,min=5.4,type='float')
        self.reg_5va=Var("5VA Rail",max=5.1,min=4.9,type='float')
        self.reg_3v3c_sb=Var("3.3VC Single Board Rail",max=3.4,min=3.2,type='float')
        self.reg_3v3c_cm1=Var("3.3VC Cell Mgmt 1 Rail",max=3.4,min=3.2,type='float')
        self.reg_3v3c_cm2=Var("3.3VC Cell Mgmt 2 Rail",max=3.4,min=3.2,type='float')
        self.reg_3v3b=Var("3.3VB Rail",max=3.4,min=3.2,type='float')
        self.reg_3v3a=Var("3.3VA Rail",max=3.4,min=3.2,type='float')
        self.reg_2v75=Var("2.75V Rail",max=2.8,min=2.7,type='float')

        self.reg_LED_lit=Var("Check if fixture LED's are lit",type='bool')
        
        self.droop_6v=Var("6V droop",max=2,min=0,type='float')
        self.droop_12v=Var("12V droop",max=1,min=0,type='float')
        self.droop_5v5=Var("5.5V droop",max=2.5,min=0,type='float')
        
        self.pump=Var("Pump test",type='bool')
        
        self.bouy_ing=Var("Ingestion motor",type='bool')
        self.fore_bouy=Var("Fore motor",type='bool')
        self.aft_bouy=Var("Aft motor",type='bool')
        
        self.pump_pressure=Var("Pump pressure",type='bool')
        self.waste_pressure=Var("Waste pressure",type='bool')
        
        self.cell_mgmt_pressure=Var("Cell Mgmt Pressure",max=12000,min=9500,type='float')
        self.cell_mgmt_temp=Var("Cell Mgmt Temp",max=2800,min=1800,type='float')
        
        self.micro_temp=Var("Micro Temp",max=30,min=15,type='float')
        self.on_board_temp=Var("PCB Temp",max=30,min=15,type='float')
        self.thermistor_temp=Var("Thermistor Temp",max=30,min=15,type='float')
        
        self.ll_1=Var("Liquid Level 1",max=[100,100,5100,5100,100,100,100],min=[-100,-100,4000,4000,-100,-100,-100],type='list')
        self.ll_2=Var("Liquid Level 2",max=[100,5100,-4000,100,5100,100,100],min=[-100,4000,-5100,-100,4000,-100,-100],type='list')
        self.ll_3=Var("Liquid Level 3",max=[5100,-4000,100,100,100,5100,100],min=[4000,-5100,-100,-100,-100,4000,-100],type='list')
        self.ll_4=Var("Liquid Level 4",max=[-4000,100,100,100,100,100,5100],min=[-5100,-100,-100,-100,-100,-100,4000],type='list')
        
        self.lc_1=Var("Load Cell 1",type='bool')
        self.lc_2=Var("Load Cell 2",type='bool')
        self.lc_3=Var("Load Cell 3",type='bool')
        self.lc_4=Var("Load Cell 4",type='bool')
        
        self.accel=Var("Accel",type='bool')
        
        self.waste_motor=Var("Waste Motor",type='bool')
        
"""
Setup portion
"""       
   
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

passing=True



DEFAULT_EXCEL_FILE_PATH="C:\\Users\\Electrical  Test\\Dropbox (Open Water Power)\\Aluminum Research\\Design and Modeling\\Electrical\\Bench Testing\\Gen2_bench_testing.xlsx"
if args.file:
    DEFAULT_EXCEL_FILE_PATH=args.file



"""
Helper Functins
"""
  
def cell_shorting(ser):
    #print("Cell Shorting: Resetting all FET's")
    for i in range (1,16):
        ser.write(("sc+cellstat="+str(i)+",0\r\n").encode())
        ser.read(9999)
        time.sleep(0)
    #print("Cell Shorting: Reset complete")
    input("Switch LED on for shorting [ENTER]")
    for i in range (1,16):
        # if i>13:
        #   input("press enter to short")
        
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
    elif current==0.5:
      load.write(bytearray([0xaa, 0, 0x2a, 0x88,0x13,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x6f]))
    elif current==1:
      load.write(bytearray([0xaa, 0, 0x2a, 0x10,0x27,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xb]))
    elif current==1.5:
      load.write(bytearray([0xaa, 0, 0x2a, 0x98,0x3a,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xa6]))
    elif current==2:
      load.write(bytearray([0xaa, 0, 0x2a, 0x20,0x4e,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x42]))
    elif current==3:
      load.write(bytearray([0xaa, 0, 0x2a, 0x30,0x75,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x79]))
    elif current==4:
      load.write(bytearray([0xaa, 0, 0x2a, 0x40,0x9c,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xB0]))
    elif current==5:
      load.write(bytearray([0xaa, 0, 0x2a, 0x50,0xC3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xE7]))
    elif current==6:
      load.write(bytearray([0xaa, 0, 0x2a, 0x60,0xEA,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x1E]))
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
    return [m,c*1000]
"""
n.b. this does a first then second search
search is exclusive (begining/end words are not in the resulting term)
"""
def getVal(target,command,start="",end=""):
    target.write((command).encode())
    time.sleep(1)
    return target.read(9999).decode().split(start)[1].split(end)[0]
     
def getVal(target,command,split_str="",split_num=0):
    target.write((command).encode())
    time.sleep(1)
    raw=target.read(9999).decode()
    #print("[RAW}:",raw)
    return raw.split(split_str)[split_num]

def power_supply_get(supply,item="VOLT"):
    supply.write("MEAS:"+item+"?")
    return float(supply.read())
def getTelemValues(target,positions,samples=1):
    target.read(9999).decode()
    target.write(("sc+telem=1\r").encode())
    val_total=[0]
    for i in range(samples):
        time.sleep(1)
        raw_val=target.read(9999).decode()
        #print(raw_val)
        try:
            val_total=[0]*len(positions)
            for j in range(len(positions)):
                val_total[j]= val_total[j]+int(raw_val.split("+telem:")[1].split(",")[positions[j]])
        except TypeError:
            val_total[0]= val_total[0]+int(raw_val.split("+telem:")[1].split(",")[positions])
    target.write(("sc+telem=0\r").encode())
    for v in val_total:
      v=v/samples
    #print(val_total)
    if len(val_total)==1:
        return val_total[0]
    return val_total
def meter_and_telem(target,meter,position,samples=1):
    meter_total=0
    telem_total=0
    for i in range(samples):
        current_meter=meter.read()
        current_telem=getTelemValues(target,position)/1000
        #print("[DEBUG] Met:",current_meter,"A  Telem:",current_telem,"A")
        meter_total=meter_total+current_meter
        telem_total=telem_total+current_telem
    return [meter_total/samples,telem_total/samples]
#SingleBoard Setup
try:
    single_board = serial.Serial(list(list_ports.grep("Ch A"))[0][0], 115200, timeout=.25)
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

pos=0
#Current Meter Setup
try:
    current_meter=U1282A(port=(list(list_ports.grep("067B:2303"))[pos][0]))
    pos=1
except:
    try:
        current_meter=U1282A(port=(list(list_ports.grep("067B:2303"))[pos+1][0]))
        pos=0
    except:
        print("Cannot find current meter/BKLoad, make sure meter is powered on")
        exit()
#BKLoad Setup
try:
    load=serial.Serial(list(list_ports.grep("067B:2303"))[pos][0], 4800, timeout=.25)
    #set remote on
    load.write(bytearray([0xaa, 0, 0x20, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
    time.sleep(0.5)
    #turn off
    load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))  

except:
    print("Unable to connect to BKLoad, try a reconnect")
    exit(1)

#Turn off telemetry
single_board.write(("sc+telem=0\r").encode())
single_board.write(("sc+lgcfg=5,5\r").encode())
single_board.read(9999)




"""
Testing portion
"""


t=Tests()
members = [attr for attr in dir(t) if not callable(getattr(t, attr)) and not attr.startswith("__")]

#Serial Number
if not args.serial:
  t.sn.setValue(int(input("Serial number:")))
else:
  t.sn.setValue(args.serial)
#UID
t.uid.setValue(getVal(single_board,"sc+uid\r\n","OK",0))

#32V output Testing
#reset to default config
single_board.write(("sc+pcfg=1,0,1,0,1,0\r\n").encode())
single_board.write(("sc+bcellthresh=200\r\n").encode())
input("    Ensure U1282A is connected to Voltage input [ENTER} >>")
single_board.write("sc+regstate=0,0\r\nsc+regstate=1,0\r\nsc+regstate=2,0\r\nsc+regstate=3,0\r\nsc+regstate=4,0\r\n".encode())

pow_supply.write("VOLTage:LEVel 15.0\n")
pow_supply.write("CURRent:LEVel 40\n")
time.sleep(0.1)
pow_supply.write("OUTPut ON\n")

t.input_32v_off.setValue([current_meter.read(),getTelemValues(single_board,1,3)/1000])
single_board.write(("sc+regstate=0,1\r\n").encode())

input_values=[0,.5,1,1.5,2]
in_value_vars=[t.input_32v_0a,t.input_32v_0a5,t.input_32v_1a,t.input_32v_1a5,t.input_32v_2a]

#set remote on
load.write(bytearray([0xaa, 0, 0x20, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
time.sleep(0.5)


set_current(load,0)
time.sleep(0.5)
#turn on
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
time.sleep(0.5)
#set cc mode
load.write(bytearray([0xaa, 0, 0x28, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xd2]))
time.sleep(0.5)

for v in range(len(input_values)):
    set_current(load,input_values[v])
    time.sleep(1)
    in_value_vars[v].setValue(meter_and_telem(single_board,current_meter,1,3))

#turn load off
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))  
pow_supply.write("OUTPut OFF\n")  


input("    Move U1282A to Voltage Output [ENTER} >>")   
#32V Output Testing
pow_supply.write("VOLTage:LEVel 7.5\n")
time.sleep(0.1)
pow_supply.write("OUTPut ON\n")    
output_values=[0,1,2,3,4,5,6]
out_value_vars=[t.output_32v_off,t.output_32v_1a,t.output_32v_2a,t.output_32v_3a,t.output_32v_4a,t.output_32v_5a,t.output_32v_6a]

#turn on
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
time.sleep(1)

for v in range(len(output_values)):
    set_current(load,output_values[v])
    time.sleep(1)
    out_value_vars[v].setValue(meter_and_telem(single_board,current_meter,3,3))

#turn load off
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))        

#turn Power Supply OFF\n
pow_supply.write("OUTPut OFF\n")

meas_input=[]
theo_input=[]
for v in in_value_vars:
  meas_input.append(v.value[0])
  theo_input.append(v.value[1])
print(theo_input)
print(meas_input)
input_cal=calc_slope_offset(theo_input,meas_input)

meas_output=[]
theo_output=[]
for v in out_value_vars:
  meas_output.append(v.value[0])
  theo_output.append(v.value[1])
print(theo_output)
print(meas_output)
output_cal=calc_slope_offset(theo_output,meas_output)


t.output_cal.setValue(output_cal)
t.input_cal.setValue(input_cal)

single_board.write(("sc+pcfg="+'%.8f' % output_cal[0]+","+'%.8f' % output_cal[1]+","+'%.8f' % input_cal[0]+","+'%.8f' % input_cal[1]+",1,0\r\n").encode())


pow_supply.write("OUTPut ON\n")
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
time.sleep(0.5)
set_current(load,3)
time.sleep(2)
t.output_3a_spot_check.setValue(getTelemValues(single_board,3,1)/1000)
#turn load off
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))        
pow_supply.write("OUTPut OFF\n")


#turn load on
single_board.write(("sc+bcellthresh=466\r\n").encode())
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
set_current(load,2)
pow_supply.write("VOLTage:LEVel 7\n")
pow_supply.write("OUTPut ON\n")
time.sleep(2)
t.boost_regulation_2a.setValue([x * .001 for x in getTelemValues(single_board,[1,2,3,5],2)]+[power_supply_get(pow_supply,"VOLT"),power_supply_get(pow_supply,"CURR")])

set_current(load,3)
time.sleep(1)
t.boost_regulation_3a.setValue([x * .001 for x in getTelemValues(single_board,[1,2,3,5],2)]+[power_supply_get(pow_supply,"VOLT"),power_supply_get(pow_supply,"CURR")])

set_current(load,4)
time.sleep(1)
t.boost_regulation_4a.setValue([x * .001 for x in getTelemValues(single_board,[1,2,3,5],2)]+[power_supply_get(pow_supply,"VOLT"),power_supply_get(pow_supply,"CURR")])

#turn load off
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))        

single_board.write("sc+regstate=0,1\r\nsc+regstate=1,1\r\nsc+regstate=2,1\r\nsc+regstate=3,1\r\nsc+regstate=4,1\r\n".encode())
 
t.reg_12v.setValue(getTelemValues(single_board,8)/1000)
t.reg_6v.setValue(getTelemValues(single_board,9)/1000)
t.reg_5v5.setValue(getTelemValues(single_board,10)/1000)
t.reg_5va.setValue(getTelemValues(single_board,11)/1000)
t.reg_3v3c_sb.setValue(getTelemValues(single_board,14)/1000)
t.reg_3v3c_cm1.setValue(getTelemValues(single_board,15)/1000)
t.reg_3v3c_cm2.setValue(getTelemValues(single_board,16)/1000)
t.reg_3v3b.setValue(getTelemValues(single_board,13)/1000)
t.reg_3v3a.setValue(getTelemValues(single_board,12)/1000)
t.reg_2v75.setValue(getTelemValues(single_board,17)/1000)

t.reg_LED_lit.setValue(input("Fixture Voltage Rail LED's Lit? ENTER=YES >>")=="")

pow_supply.write("VOLTage:LEVel 3\n")
t.droop_6v.setValue(getTelemValues(single_board,9)/1000)
 
pow_supply.write("VOLTage:LEVel 2.5\n") 
t.droop_12v.setValue(getTelemValues(single_board,8)/1000) 
 
pow_supply.write("VOLTage:LEVel 0.75\n") 
t.droop_5v5.setValue(getTelemValues(single_board,10)/1000) 

pow_supply.write("VOLTage:LEVel 7.5\n")




#set open loop
single_board.write("sc+mimode=0\r\n".encode())
input("    Set Motor switch to Ingestion [ENTER} >>")
single_board.write("sc+mion=1\r\n".encode())
ing_passing=(input("Motor Spinning? Enter=YES>>")=="")
single_board.write("sc+mion=0\r\n".encode())
ing_passing=(input("Motor Stopped? Enter=YES>>")=="") and ing_passing
t.bouy_ing.setValue(ing_passing)

motors=[t.fore_bouy,t.aft_bouy]
for m in motors:
    input("    Set Motor switch to "+m.name+" [ENTER} >>")
    single_board.write(("sc+bymotor="+str(motors.index(m))+",1\r\n").encode())
    motor_passing=(input("Motor Spinning? Enter=YES>>")=="")
    single_board.write(("sc+bymotor="+str(motors.index(m))+",2\r\n").encode())
    motor_passing=(input("Motor Spinning in Reverse? Enter=YES>>")=="")
    single_board.write(("sc+bymotor="+str(motors.index(m))+",0\r\n").encode())
    motor_passing=(input("Motor Stopped? Enter=YES>>")=="") and motor_passing
    m.setValue(motor_passing)

    
#pump
single_board.write("sc+mpvel=2000\r\n".encode())
pump_passing=(input("Pump Spinning CCW & up to speed?  Enter=YES>>")=="")
#print(pump_passing)
pump_math=abs(2000-getTelemValues(single_board,116,1))
#print("MATH:",pump_math)
pump_passing=pump_math<300 and pump_passing
#print(pump_passing)
single_board.write("sc+mpenable=0\r\n".encode())
pump_passing=(input("Pump Stopped? Enter=YES>>")=="") and pump_passing
#print(pump_passing)
single_board.write("sc+mpenable=1\r\nsc+mpvel=-2000\r\n".encode())
pump_passing=(input("Pump Spinning CWEnter=YES>>")=="") and pump_passing
#print(pump_passing)

t.pump.setValue(pump_passing)
    
    
#pressure sensors
single_board.write("sc+pscfg=1,0,1,0\r\n".encode())
psensors=[t.pump_pressure,t.waste_pressure]
vals=[138,139]
for p in psensors:
    input("    Set Pressure Sensor to "+p.name+" [ENTER} >>")
    pre=[getTelemValues(single_board,vals[psensors.index(p)],1),getTelemValues(single_board,vals[psensors.index(p)],1),getTelemValues(single_board,vals[psensors.index(p)],1)]
    pre_std=statistics.stdev(pre)
    pre_avg=sum(pre)/len(pre)
    input("Excite pressure sensor>>")
    post=[getTelemValues(single_board,vals[psensors.index(p)],1),getTelemValues(single_board,vals[psensors.index(p)],1),getTelemValues(single_board,vals[psensors.index(p)],1)]
    post_std=statistics.stdev(post)
    post_avg=sum(post)/len(post)
    print("    Pre dev=",pre_std," Post dev=",post_std,"\nPre avg=",pre_avg,"Post avg=",post_avg)
    p.setValue(pre_std<=post_std)

t.cell_mgmt_pressure.setValue(getTelemValues(single_board,83,1))
t.cell_mgmt_temp.setValue(getTelemValues(single_board,84,1))

#not actually checking this
t.micro_temp.setValue(25)

t.on_board_temp.setValue(getTelemValues(single_board,125,1)/100)

t.thermistor_temp.setValue(getTelemValues(single_board,123,1)/100)  

ll=[t.ll_1,t.ll_2,t.ll_3,t.ll_4]  

input("    Set Liquid level switch to 3 o'clock [ENTER} >>")
for l in ll:
    input("Set switch to "+l.name+" >>")
    time.sleep(4)
    l.setValue(getTelemValues(single_board,[93,94,95,106,105,104,103],1))      

lc=[t.lc_1,t.lc_2,t.lc_3,t.lc_4]
vals=[111,112,113,114]
for l in lc:
    input("    Set switch to "+l.name+" [ENTER} >>")
    pre=getTelemValues(single_board,vals[lc.index(l)],1)
    input("    Excite load cell sensor>>")
    post=getTelemValues(single_board,vals[lc.index(l)],3)
    print("Pre=",pre," Post=",post)
    l.setValue(post<=pre)
    
#not actually checking this
t.accel.setValue(1)
pow_supply.write("OUTPut ON\n")       
input("    Moving savox [ENTER} >>")
single_board.write("sc+mwpos=1000\r\n".encode())
t.waste_motor.setValue(input("    Savox moved? Enter=YES >>")=="")
 
pow_supply.write("OUTPut OFF\n")

ask=input("    Save to file? (y/n)")
if ask == 'y' or ask == 'Y':
    x=Excel(DEFAULT_EXCEL_FILE_PATH)
    x.writeToFile([serial_number,measured_current,measured_volts,led_test,pressure_test],is_primary)
    x.saveFile()
else:
    pass
single_board.write(("sc+telem=1\r").encode())
single_board.write(("sc+lgcfg=1,1\r").encode())
pow_supply.close()
exit(1)