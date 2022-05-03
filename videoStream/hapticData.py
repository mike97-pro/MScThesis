import serial

Frated = 5  # N
RES = 0x4000
OUTmax = 0.8 * RES  # 80% of 2^14
OUTmin = 0.2 * RES  # 20% of 2^14
bias = 0.0
first = True

tau = 0.078  # N*m max motor torque
r = 0.0055  # m pulley radius
ks = 300  # N/m spring stiffness
Kf = 500
d0 = 0.015  # m spring length


min_th = 178.55
max_th = 180

class HapticSerial:

    def __init__(self, ttyStr, baud):
        try:
            self.usb = serial.Serial(ttyStr, baud)
        except:
            self.usb=None
        self.port = ttyStr
        self.baud = baud

    def readData(self):
        try:
            if self.usb is None:
                self.usb = serial.Serial(self.port, self.baud)
            data = str(self.usb.readline())
            data = data[2:-5]
            data = data.split(' ')
            return [int(i) for i in data]
        except:
            if self.usb is not None:
                self.close()
                self.usb = None
            return None

    def writeData(self, data):
        try:
            if self.usb is None:
                self.usb = serial.Serial(self.port, self.baud)
            self.usb.write(data)
        except:
            if self.usb is not None:
                self.close()
                self.usb = None
            print('Serial write error')

    def writeAngle(self, th):
        byte = int(abs(th-min_th)*255/(max_th-min_th))
        if byte > 255:
            byte = 255
        byte = byte.to_bytes(1, 'little')
        self.writeData(byte)

    def writeForce(self, reading, th0):
        global bias, first

        cnt = (reading[0] + reading[1] + reading[2] + reading[3])
        F = ((cnt - OUTmin) * Frated / (OUTmax - OUTmin) - bias)
        
        if first:
            bias = F
            F -= bias
            first = False

        th = th0 - F / (r * Kf)

        self.writeAngle(th)

    def close(self):
        self.usb.close()

    def __del__(self):
        if self.usb:
            self.usb.close()
