from asyncio import protocols
import keyboard
import serial
import time
port = 'COM14'
baudrate = 115200
serialPort = serial.Serial(port=port, baudrate=baudrate,
                                bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
serialString = ""
def CheckSumCal(sum):
    
    # print(sum)
    # print(hex(sum))
    # print(bin(sum))
    sumbin=bin(sum)[2:]
    # print(sumbin)
    if len(sumbin)>8:
        sumbin=sumbin[-8:]
    elif len(sumbin)<8:
        sumbin = '0'*(8-len(sumbin)) + sumbin
    # print(sumbin)
    onecom=''
    for i in sumbin:
        if i=='0':
            i='1'
        else:
            i='0'
        onecom=onecom+i
    # print('1s',onecom)
    twocom = int(onecom,2) + 1
    # print('2s',twocom)
    out = []
    for i in str(hex(twocom)[2:]):
        # print(i)
        # print(hex(ord(i)))
        out.append(hex(ord(i))[2:])
    return out

def acknowledge():
    arr = []
    arr = serialPort.read(5)
    if len(arr)>0:
        print('ss',arr[0],arr[1],arr[2],arr[3],arr[4])
        sum = 0
        for i in arr[1:-2]:
            # print(i)
            sum=sum+i
        checksum = CheckSumCal(sum)
        # print(checksum)
        if(arr[0] == 255):
            if(arr[1] == 1):
                # print(hex(arr[-2])[2:],hex(arr[-1])[2:])
                if(hex(arr[-2])[2:] == checksum[0] and hex(arr[-1])[2:] == checksum[1]):
                    print('ss',time.time())
                    # serialPort.close()
                else:
                    arr = []
            else:
                arr = []
        else:
            arr = []
        arr = []
    else:
        arr = []

def jointjogProtocol(Instruction):
    a = ['01','01','01',Instruction]
    serialPort.write(bytes.fromhex('FF'))
    serialPort.write(bytes.fromhex(a[0]))
    serialPort.write(bytes.fromhex(a[1]))
    serialPort.write(bytes.fromhex(a[2]))
    serialPort.write(bytes.fromhex(a[3]))
    sum=0
    for i in a:
        sum = sum+int(i, 16)
    # print(CheckSumCal(a))
    serialPort.write(bytes.fromhex(CheckSumCal(sum)[0]))
    serialPort.write(bytes.fromhex(CheckSumCal(sum)[1]))
def KeyPressSerial():
    x=keyboard.read_key()
    if x == "r": #72
        print('r')
        jointjogProtocol('72')
        acknowledge()
        #serialPort.write(bytes.fromhex("01"))
    elif x == "f": #66
        print('f')
        jointjogProtocol('66')
        acknowledge()
        # serialPort.write(bytes.fromhex("02"))
    elif x == "t": #74
        print('t')
        jointjogProtocol('74')
        acknowledge()
        # serialPort.write(bytes.fromhex("03")) 
    elif x == "g": #67
        print('g')
        jointjogProtocol('67')
        acknowledge()
        # serialPort.write(bytes.fromhex("04"))
    elif x == "y": #79
        print('y')
        jointjogProtocol('79')
        acknowledge()
        # serialPort.write(bytes.fromhex("05")) 
    elif x == "h": #41
        print('h')
        jointjogProtocol('41')
        acknowledge()
        # serialPort.write(bytes.fromhex("06"))
    elif x == "u": #75
        print('u')
        jointjogProtocol('75')
        acknowledge()
        # serialPort.write(bytes.fromhex("07"))  
    elif x == "j": #6A
        print('j')
        jointjogProtocol('6A')
        acknowledge()
        # serialPort.write(bytes.fromhex("08"))          

while True:
    KeyPressSerial()
    # time.sleep(0.1)
    # serialPort.reset_input_buffer()
    # serialPort.reset_output_buffer()
    # serialString = serialPort.read().hex()
    # serialString = serialPort.read()
    # print(serialString)
    # if serialString != "":
        # print(serialString)
        # break

serialPort.close()
