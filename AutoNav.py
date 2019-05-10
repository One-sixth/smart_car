import numpy as np
from car_op import CarOp
from threading import Thread
import time


class AutoNav:

    need_quit = False
    need_wait = True

    current_status = None
    location = 'any'

    def __init__(self, car_op: CarOp = None):
        self.car_op = car_op
        self.nav_thread = Thread(target=self.nav_run)
        self.nav_thread.start()

    def start(self):
        self.need_wait = False

    def stop(self):
        self.need_wait = True

    def nav_run(self):
        while not self.need_quit:
            time.sleep(0.5)
            if self.need_wait or self.car_op.offline:
                time.sleep(1)
                continue

            if self.location == 'any':
                # 随机运动
                if self.car_op.ultrasonic_radar_status[1]:
                    print('AutoNav: forwarding')
                    self.car_op.move_forward()
                else:
                    print('AutoNav: found forward blocked')
                    self.car_op.move_stop()
                    if self.car_op.ultrasonic_radar_status[2]:
                        print('AutoNav: try rotate to right')
                        self.car_op.move_back()
                        time.sleep(1.)
                        self.car_op.move_right()
                        time.sleep(1.)
                        self.car_op.move_stop()
                    elif self.car_op.ultrasonic_radar_status[0]:
                        print('AutoNav: found right blocked')
                        print('AutoNav: try rotate to left')
                        self.car_op.move_back()
                        time.sleep(1.)
                        self.car_op.move_left()
                        time.sleep(1.)
                        self.car_op.move_stop()
                    else:
                        print('AutoNav: try move back')
                        self.car_op.move_back()
                        time.sleep(1.)

            elif self.location == 'neighbor':
                # 靠边停下

                # 找到尽头
                while not self.need_wait and not self.car_op.offline:
                    print('AutoNav: forwarding')
                    self.car_op.move_forward()
                    time.sleep(0.5)
                    if not self.car_op.ultrasonic_radar_status[1]:
                        break

                # 旋转直到车头无障碍物
                while not self.need_wait and not self.car_op.offline:
                    print('AutoNav: forwarding')
                    self.car_op.move_back()
                    time.sleep(1.)
                    self.car_op.move_right()
                    time.sleep(0.5)
                    if self.car_op.ultrasonic_radar_status[1]:
                        break

                # 停下
                self.car_op.move_stop()
                self.location = 'none'

    def to(self, location='any'):
        if location == 'any':
            self.location = location
        elif location == 'neighbor':
            self.location = location
        elif location == 'none':
            self.location = location
        else:
            print('unknow location')


if __name__ == '__main__':
    nav = AutoNav(None)
