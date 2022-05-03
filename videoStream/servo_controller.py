import maestro

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


class Attuator:
    def __init__(self, ttyStr, tik):
        self.servo = maestro.Controller(ttyStr=ttyStr)
        self.servo.setAccel(0, 100)  # set servo 0 acceleration to 4
        self.servo.setSpeed(0, 100)  # set speed of servo
        self.servo.setRange(0, 4000, 8000)
        self.servo.setRangeDeg(0, 178.55, 180)
        self.th0 = (tik - 4000) / 4000 * (0.008 / r) + (180 - 0.008 / r)
        # servo.setTarget(0, int(sys.argv[2]))
        self.servo.setTargetDegree(0, self.th0)

        # d1 = d0 - r * (180 - th0)
        # Ff0 = tau / r - 3 * ks * (d0 - d1)

    def appForce(self, data):
        global bias, first
        cnt = (data[0] + data[1] + data[2] + data[3])
        F = ((cnt - OUTmin) * Frated / (OUTmax - OUTmin) - bias)

        if first:
            bias = F
            F -= bias
            first = False

        th = self.th0 - F / (r * Kf)

        if not self.servo.isMoving(0):
            self.servo.setTargetDegree(0, th)

        '''if x%2 == 0:
            if not servo.isMoving(0):
                servo.setTarget(0, 9000)
        else:
            if not servo.isMoving(0):
                servo.setTarget(0, 4000)'''
