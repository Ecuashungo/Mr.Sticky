import serial
import time

class Arduino():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        c_recu = self.ser.read(1)
        while ord(c_recu) != 0:
            c_recu = self.ser.read(1)
        c_recu = self.ser.read(1)
        while ord(c_recu) != 255:
            c_recu = self.ser.read(1)
        c_recu = self.ser.read(1)
        while ord(c_recu) != 0:
            c_recu = self.ser.read(1)
        self.ANALOG_WRITE = 100
        self.ANALOG_READ = 101

    def close(self):
        self.ser.close()

    def sendToArduino(self, angle_target):
        self.ser.write(chr(100))
        self.ser.write(chr(angle_target))

    def receiveFromArduino(self, pin):
        ID = ord(self.ser.read(1))
        value = ord(self.ser.read(1))
        return ID * 0x100 + value
