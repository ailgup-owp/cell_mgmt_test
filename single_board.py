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
            #in the future give option to save
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
        
        self.boost_regulation_2a=Var("Input Voltage Regulation, 2A",max=[6.5,7,2.2,32,7.1,6.6],min=[6,6.5,1.8,15,6.9,6],type='list')
        self.boost_regulation_3a=Var("Input Voltage Regulation, 3A",max=[6.7,7,3.2,32,7.1,6.8],min=[6.1,6.5,2.8,7,6.9,6.2],type='list')
        self.boost_regulation_4a=Var("Input Voltage Regulation, 4A",max=[6.9,7,4.2,15,7.1,7],min=[6.2,6.5,3.8,7,6.9,6.4],type='list')
        
        self.reg_12v=Var("12V Rail",max=12.2,min=11.8,type='float')
        self.reg_6v=Var("6V Rail",max=6.1,min=5.88,type='float')
        self.reg_5v5=Var("5.5V Rail",max=5.6,min=5.4,type='float')
        self.reg_5va=Var("5VA Rail",max=5.1,min=4.9,type='float')
        self.reg_3v3c_sb=Var("3.3VC Single Board Rail",max=3.2,min=3.4,type='float')
        self.reg_3v3c_cm1=Var("3.3VC Cell Mgmt 1 Rail",max=3.2,min=3.4,type='float')
        self.reg_3v3c_cm2=Var("3.3VC Cell Mgmt 2 Rail",max=3.2,min=3.4,type='float')
        self.reg_3v3b=Var("3.3VB Rail",max=3.2,min=3.4,type='float')
        self.reg_3v3a=Var("3.3VA Rail",max=3.2,min=3.4,type='float')
        self.reg_2v75=Var("2.75V Rail",max=2.7,min=2.8,type='float')

        self.reg_LED_lit("Check if fixture LED's are lit",type=bool)
        
        self.droop_6v=Var("6V droop",max=1,min=0,type='float')
        self.droop_12v=Var("12V droop",max=1,min=0,type='float')
        self.droop_5v5=Var("5.5V droop",max=1,min=0,type='float')
        
        self.pump=Var("Pump test",type='bool')
        
        self.bouy_ing=Var("Ingestion motor",type='bool')
        self.fore_bouy=Var("Fore motor",type='bool')
        self.aft_bouy=Var("Aft motor",type='bool')
        
        self.pump_pressure=Var("Pump pressure",type='bool')
        self.waste_pressure=Var("Waste pressure",type='bool')
        
        self.cell_mgmt_pressure_temp=Var("Cell Mgmt Pressure/Temp",type='bool')
        self.micro_temp=Var("Micro Temp",max=30,min=15,type='float')
        self.on_board_temp=Var("PCB Temp",max=30,min=15,type='float')
        self.thermistor_temp=Var("Thermistor Temp",max=30,min=15,type='float')
        
        self.ll_1=Var("Liquid Level 1",max=[.1,.1,5.1,5.1,.1,.1,.1],min=[-.1,-.1,4,4,-.1,-.1,-.1],type='list')
        self.ll_2=Var("Liquid Level 2",max=[.1,5.1,-4.7,.1,5.1,.1,.1],min=[-.1,4,-5.1,.1,4,-.1,-.1],type='list')
        self.ll_3=Var("Liquid Level 3",max=[5.1,-4.7,.1,.1,.1,5.1,.1],min=[4,-5.1,-.1,-.1,-.1,4,-.1],type='list')
        self.ll_4=Var("Liquid Level 4",max=[-4.7,.1,.1,.1,.1,.1,5.1],min=[-5.1,-.1,-.1,-.1,-.1,-.1,4],type='list')
        
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
    return [m,c]
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
    return target.read(9999).decode().split(split_str)[split_num]

def power_supply_get(supply,item="VOLT"):
    supply.write("MEAS:"+item+"?")
    return float(supply.read())
def getTelemValues(target,positions,samples=1):
    target.read(9999).decode()
    target.write(("sc+telem=1\r").encode())
    val_total=[]
    for i in range(samples):
        time.sleep(1)
        raw_val=target.read(9999).decode()
        #print(raw_val)
        try:
            for j in range(len(positions)):
                val_total[j]= val_total[j]+int(raw_val.split("+telem:")[1].split(",")[positions[j]])
        except TypeError:
            val_total[0]= val_total[0]+int(raw_val.split("+telem:")[1].split(",")[positions])
    target.write(("sc+telem=0\r").encode())
    time.sleep(0.5)
    for v in val_total:
      v=v/samples
    if len(val_total)==1:
        return val_total[0]
    return val_total
def meter_and_telem(target,meter,position,samples=1):
    meter_total=0
    telem_total=0
    for i in range(samples):
        meter_total=meter_total+meter.read()
        telem_total=telem_total+getTelemValues(target,position)/1000
    return [meter_total/samples,telem_total/samples]
#SingleBoard Setup
try:
    single_board = serial.Serial(list(list_ports.grep("04E2:1422"))[0][0], 115200, timeout=.25)
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
        raise Exception("Cannot find current meter/BKLoad")
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
t.uid.setValue(getVal(single_board,"sc+uid\r","OK",0))

#32V output Testing
#reset to default config
single_board.write(("sc+pcfg=1,0,1,0,1,0\r\n").encode())

input("    Ensure U1282A is connected to Voltage input")
single_board.write("sc+regstate=0,0\r\nsc+regstate=1,0\r\nsc+regstate=2,0\r\nsc+regstate=3,0\r\nsc+regstate=4,0\r\n".encode())

pow_supply.write("VOLTage:LEVel 15.0\n")
pow_supply.write("CURRent:LEVel 30\n")
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
    time.sleep(0.5)
    in_value_vars[v].setValue(meter_and_telem(single_board,current_meter,1,3))

#turn load off
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))  
pow_supply.write("OUTPut OFF\n")  


input("    Move U1282A to Voltage Output")   
#32V Output Testing
pow_supply.write("VOLTage:LEVel 7.5\n")
time.sleep(0.1)
pow_supply.write("OUTPut ON\n")    
output_values=[0,1,2,3,4,5,6]
out_value_vars=[t.output_32v_off,t.output_32v_1a,t.output_32v_2a,t.output_32v_3a,t.output_32v_4a,t.output_32v_5a,t.output_32v_6a]

#turn on
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
time.sleep(0.5)

for v in range(len(output_values)):
    set_current(load,output_values[v])
    time.sleep(0.5)
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

single_board.write(("sc+pcfg="+output_cal[0]+","+output_cal[1]+","+input_cal[0]+","+input_cal[1]+",1,0\r\n").encode())

pow_supply.write("OUTPut ON\n")
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
time.sleep(0.5)
set_current(load,3)
t.output_3a_spot_check.setValue(getTelemValues(single_board,3)/1000)
#turn load off
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))        
pow_supply.write("OUTPut OFF\n")


#turn load on
single_board.write(("sc+bcellthresh=4660\r\n").encode())
load.write(bytearray([0xaa, 0, 0x21, 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcc]))
set_current(load,2)
pow_supply.write("VOLTage:LEVel 7\n")
pow_supply.write("OUTPut ON\n")
t.boost_regulation_2a.setValue([getTelemValues(single_board,[1,2,3,5])+[power_supply_get(pow_supply,"VOLT"),power_supply_get(pow_supply,"CURR")])

set_current(load,3)
t.boost_regulation_3a.setValue([getTelemValues(single_board,[1,2,3,5])+[power_supply_get(pow_supply,"VOLT"),power_supply_get(pow_supply,"CURR")])

set_current(load,4)
t.boost_regulation_4a.setValue([getTelemValues(single_board,[1,2,3,5])+[power_supply_get(pow_supply,"VOLT"),power_supply_get(pow_supply,"CURR")])

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


pump_passing=(input("Pump Spinning CCW? Enter=YES>>")=="")

single_board.write("sc+mpvel=0\r\n".encode())
pump_passing=(input("Pump Stopped? Enter=YES>>")=="") and pump_passing

single_board.write("sc+mpvel=-2000\r\n".encode())
pump_passing=(input("Pump Spinning CW? Enter=YES>>")=="") and pump_passing

time.sleep(1)
pump_passing=(abs(2000+getTelemValues(single_board,116,3))<300) and pump_passing
t.pump.setValue(pump_passing)

#set open loop
single_board.write("sc+mimode=0\r\b".encode())
input("Set Motor switch to Ingestion >>")
single_board.write("sc+mion=1\r\b".encode())
ing_passing=(input("Motor Spinning? Enter=YES>>")=="")
single_board.write("sc+mion=0\r\b".encode())
ing_passing=(input("Motor Stopped? Enter=YES>>")=="") and ing_passing
t.bouy_ing.setValue(ing_passing)

motors=[t.fore_bouy,t.aft_bouy]
for m in motors:
    input("Set Motor switch to "+m.name+" >>")
    single_board.write("sc+bymotor="+motors.index(m)+",1\r\b".encode())
    motor_passing=(input("Motor Spinning? Enter=YES>>")=="")
    single_board.write("sc+bymotor="+motors.index(m)+",2\r\b".encode())
    motor_passing=(input("Motor Spinning in Reverse? Enter=YES>>")=="")
    single_board.write("sc+bymotor="+motors.index(m)+",0\r\b".encode())
    motor_passing=(input("Motor Stopped? Enter=YES>>")=="") and motor_passing
    m.setValue(motor_passing)

        
        self.pump_pressure=Var("Pump pressure",type='bool')
        self.waste_pressure=Var("Waste pressure",type='bool')
        
        self.cell_mgmt_pressure_temp=Var("Cell Mgmt Pressure/Temp",type='bool')
        self.micro_temp=Var("Micro Temp",max=30,min=15,type='float')
        self.on_board_temp=Var("PCB Temp",max=30,min=15,type='float')
        self.thermistor_temp=Var("Thermistor Temp",max=30,min=15,type='float')
        
        self.ll_1=Var("Liquid Level 1",max=[.1,.1,5.1,5.1,.1,.1,.1],min=[-.1,-.1,4,4,-.1,-.1,-.1],type='list')
        self.ll_2=Var("Liquid Level 2",max=[.1,5.1,-4.7,.1,5.1,.1,.1],min=[-.1,4,-5.1,.1,4,-.1,-.1],type='list')
        self.ll_3=Var("Liquid Level 3",max=[5.1,-4.7,.1,.1,.1,5.1,.1],min=[4,-5.1,-.1,-.1,-.1,4,-.1],type='list')
        self.ll_4=Var("Liquid Level 4",max=[-4.7,.1,.1,.1,.1,.1,5.1],min=[-5.1,-.1,-.1,-.1,-.1,-.1,4],type='list')
        
        self.lc_1=Var("Load Cell 1",type='bool')
        self.lc_2=Var("Load Cell 2",type='bool')
        self.lc_3=Var("Load Cell 3",type='bool')
        self.lc_4=Var("Load Cell 4",type='bool')
        
        self.accel=Var("Accel",type='bool')
        
        self.waste_motor=Var("Waste Motor",type='bool') 
 
 
pow_supply.write("VOLTage:LEVel 5.0\n")
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
  if (i>1 and (abs((measured_current[-1]-(1000*i))/(1000*i))>.25)):
    print("FAILED!! Current sense is far off from calibration, measured:",measured_current[-1]/1000," where ",i," was expected")
    load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
    load.write(bytearray([0xaa, 0, 0x20, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xca]))
    pow_supply.write("OUTPut OFF\n")
    exit(1)
  print("Current Calibration: Measured current= ",measured_current[-1],"mA")
#turn off &remove remote
load.write(bytearray([0xaa, 0, 0x21, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xcb]))
time.sleep(.5)
load.write(bytearray([0xaa, 0, 0x20, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xca]))
pow_supply.write("OUTPut OFF\n")
[current_slope,current_offset]=calc_slope_offset(current_steps,measured_current)
print("Current Calibration: ",current_slope,"  ",current_offset)
lj.setDIOState(16,0)
###Prompt removal of HV Connector


"""
LJ Voltage Check
"""
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
                if tries==4:
                    measured_volts.append(val)
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
            print("FAILED!!: Cell Voltage ",offending_value," not read correctly on cell #",measured_volts.index(offending_value)+1)
            print(measured_volts)
            ask=input("Try again? (y/n)")
            if ask is "n":
                break            
        else:
            print("Measured Voltages: ",measured_volts)
            cont=False
    lj.i2c(0x48, [0x00], SDAPinNum=14, SCLPinNum=15)
    lj.i2c(0x49, [0x00], SDAPinNum=14, SCLPinNum=15)



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
pow_supply.close()
exit(1)