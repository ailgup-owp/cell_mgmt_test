import serial
from serial.tools import list_ports
import time

""" TODO
add reset, get max and min

"""

class U1282A:
    def __init__(self,port=None):
        if port is None:
            #look for a port
            portList=list_ports.comports()
            for p in portList:
                if "Prolific USB-to-Serial Comm Port" in p[1]:
                    port=p[0]
                    break
            if port is None:
                raise Exception("Unable to find device")
                
        self.port=port
        
        self.open()
        try:
            self.getMode()
        except:
            self.close()
            raise
    def open(self):
        self.ser = serial.Serial(self.port, 9600, timeout=1)
    def close(self):
        self.ser.close()
    def reset(self):
        while True:
            self.ser.write(("*RST\r\n").encode())
            k=self.ser.read(999)
            if k != None:
                print("Reset",k)
                return
    def read(self,count=1):
        total=0
        for i in range(count):
            self.ser.write(("READ?\r\n").encode())
            k=self.ser.read(999)
            if k != None:
                #print(float(k),self.unit)
                total=total+ float(k)
            time.sleep(0.1)
        return total/count
    def getMode(self):
        modes=["ACV measurement","ACmV measurement","ACDCV measurement",
                "ACDCmV measurement","Ω/nS measurement","Diode measurement",
                "CAP/temperature measurement","μA/mA measurement","Current measurement",
                "SQU output"]
        units=["V","mV","V","mV","Ω","V","F","uA/mA","A",""]
        while True:
            self.ser.write(("STAT?\r\n").encode())
            k=str(self.ser.read(999).decode().split("\"")[1]).strip("\"")
            #print("k:",k)
            if k != None:
                mode=modes[int(k[15])]
                self.mode=mode
                self.unit=units[int(k[15])]
                return mode
        
    def getSerial(self):
        while True:
            self.ser.write(("*IDN\r\n").encode())
            k=str(self.ser.read(999).decode())
            print(k)
            if "U1282A" in k:
                pass    
            else:
                raise
            if k != None:
                val= k.split(",")[2]
                return val

