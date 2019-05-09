import RPi.GPIO as io
from threading import Thread
import math
import time
from UltrasoundRanger import UltrasoundRanger


class UltrasonicRadar:

    motor_pin = 35

    angle_left = [2]
    angle_forward = [1]
    angle_right = [0]

    angles = [9, 14, 19]
    angles_len = [0 for _ in angles]

    # 6cm
    can_run_len = 12

    need_quit = False

    def __init__(self, *, no_run=False):
        io.setmode(io.BOARD)
        io.setup(self.motor_pin, io.OUT)
        self.motor = io.PWM(self.motor_pin, 100)
        self.ur = UltrasoundRanger()
        if no_run:
            self.motor.start(11)
        else:
            self.auto_check_thread = Thread(target=self.auto_check_run)
            self.auto_check_thread.start()

    def check_forward(self):
        return self.check_range(self.angle_forward)

    def check_left(self):
        return self.check_range(self.angle_left)

    def check_right(self):
        return self.check_range(self.angle_right)

    def check_range(self, range_):
        b = True
        for i in range_:
            b &= self.angles_len[i] > self.can_run_len
        return b

    def check_angle(self, a):
        self.motor.ChangeDutyCycle(a)
        time.sleep(0.1)
        le = self.ur.detect()
        return le

    def auto_check_run(self):
        self.motor.start(self.angles[int(math.ceil(len(self.angles)/2))])
        while not self.need_quit:
            for i, a in enumerate(self.angles):
                if i == 0:
                    continue
                self.motor.ChangeDutyCycle(a)
                time.sleep(0.15)
                le = self.ur.detect()
                if le is None:
                    le = 99999
                self.angles_len[i] = le
                time.sleep(0.1)

            for i, a in enumerate(self.angles[::-1]):
                if i == 0:
                    continue
                i = len(self.angles_len) - i - 1
                self.motor.ChangeDutyCycle(a)
                time.sleep(0.15)
                le = self.ur.detect()
                if le is None:
                    le = 99999
                self.angles_len[i] = le
                time.sleep(0.1)

if __name__ == '__main__':
    ur = UltrasonicRadar()
    while True:
        print(ur.angles_len)
        time.sleep(1)

    # ur = UltrasonicRadar(no_run=True)
    # while True:
    #     a = input('angle: ')
    #     a = int(a)
    #     print(ur.check_angle(a))
