import serial
import time
from serial.tools import list_ports

class REPLY:
  BAD_CHECKSUM = 0x90
  BAD_PARAMETER = 0xA0
  UNRECOGNIZED_COMMAND = 0xB0
  INVALID_COMMAND = 0xC0
  SUCCESS = 0x80

class BKLoad:

  def csum(self,thing):
    sum = 0;
    for i in range(len(thing)):
        sum+=thing[i]
    return 0xFF&sum;
  def setCC(self):
    self.send_command(0x28,0x0)
  def on(self):
    self.send_command(0x21,0x1)
  def off(self):
    self.send_command(0x21,0x0)
  def remote(self):
    self.send_command(0x20,0x1)
  def remote_off(self):
    self.send_command(0x20,0x0)
  def set_current(self,current):
    self.send_command(0x2A,current*10000)
    
  def send_command(self,cmd,cmd_data,try_num=1):
    byte_str=0xAA # Required byte 0
    byte_str = byte_str | (self.addr<<(1*8)) #address byte
    byte_str = byte_str | (cmd<<(2*8)) #command byte
    byte_str = byte_str | (cmd_data<<((3)*8)) #command byte 
    byte_str = byte_str | (self.csum(byte_str.to_bytes(25, byteorder='little'))<<(25*8))
    print("SENT)",byte_str.to_bytes(26, byteorder='little'))
    self.load.write(byte_str.to_bytes(26, byteorder='little'))
    reply=self.load.read_until()
    print("RE)",reply)
    print("RE)",reply[3])
    if ( reply[2] == 0x12 and reply[3] == REPLY.SUCCESS):
        #success
        print("SUCCESS")
    elif try_num>5:
        raise Exception("Unable to send command")
    else:
        self.send_command(cmd,cmd_data,try_num+1)
    
  def verify_command(self,cmd):
    pass
  def read_value(self,cmd):
    byte_str=0xAA # Required byte 0
    byte_str = byte_str | (self.addr<<(1*8)) #address byte
    byte_str = byte_str | (cmd<<(2*8)) #command byte 
    byte_str = byte_str | (self.csum(byte_str.to_bytes(25, byteorder='little'))<<(25*8))
    print("SENT)",byte_str.to_bytes(26, byteorder='little'))
    self.load.write(byte_str.to_bytes(26, byteorder='little'))
    reply=self.load.read_until()
    val=int.from_bytes(reply[3:24], byteorder='little')
    print(val)
  
  def close(self):
    self.load.close()
  
  def __init__(self,addr=0):
    self.addr=addr
    try:
        self.load=serial.Serial(list(list_ports.grep("067B:2303"))[0][0], 4800, timeout=.25)
        self.load.read()
    except:
        print("Unable to connect to BKLoad, try a reconnect")
        exit(1)

