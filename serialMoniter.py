import serial
import time

baud = 115200
port = "/dev/ttyACM0"
serialDevice = None


"""
    init should be called automatically on import 
"""

def init():
    global serialDevice
    serialDevice = serial.Serial(port, baud, timeout = 1.0)



"""
    reads data off of the microcontroller
    format:
    [0] = X-axis rotational velocity
    [1] = Y-axis rotational velocity
    [2] = Z-axis rotational velocity
    [3] = time between samples in microseconds

    this may report 0's for up to 10 seconds as the IMU controller connects
    to the access point. 
    
"""
def getData():
    global serialDevice
    data = serialDevice.readline().decode('utf-8').rstrip().split(",")
    if data == ['']:
        data = [0, 0, 0, 0]
    else:
        data[0] = float(data[0][2:])
        data[1] = float(data[1][2:])
        data[2] = float(data[2][2:])
        data[3] = int(data[3][3:])
    return data




def main():
    while True:
        data = getData()
        print(data)



init()
if __name__ == "__main__":
    main()