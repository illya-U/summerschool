import serial


serialPort = serial.Serial(port = "COM5", baudrate = 115200, bytesize = 8, stopbits = serial.STOPBITS_ONE)

def wrapper():
    while(serialPort.in_waiting == 0):
        {}
    while (serialPort.in_waiting != 0):
        serialString = serialPort.readline()
    serialString = serialString.decode("Ascii")
    return serialString

def  temperature_UART():
        serialString = wrapper()
        serialString = serialString.split(",")[0]
        serialString = serialString.strip("/*")
        return serialString


def presure_UART():
        serialString = wrapper()
        serialString = serialString.split(",")[1]
        serialString = serialString.strip("*/\n")
        return serialString


def All_UART():
        serialString = wrapper()
        serialString_1 = serialString.split(",")[0]
        serialString_2 = serialString.split(",")[1]
        serialString_1 = serialString_1.strip("/*")
        serialString_2 = serialString_2.strip("*/\n")
        return [serialString_1,serialString_2]