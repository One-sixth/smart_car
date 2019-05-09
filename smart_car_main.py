import time

import cv2
import numpy as np
from AutoNav import AutoNav
from car_op import CarOp
from orbslam2_connector import OrbSlam2Connector


class SmartCar:
    # 自动模式
    auto_mode = True

    current_status = ''

    jpeg_quality = 80

    def __init__(self):
        self.car_op = CarOp()
        self.auto_nav_er = AutoNav(self.car_op)
        # self.orbslam2_connect = OrbSlam2Connector()

    def what_i_need_do(self):
        p = np.random.uniform(0, 1)
        if p < 0.1:
            a = 'patrol'
        elif p > 0.9:
            a = 'stop'
        else:
            a = 'idle'
        return a

    def ctrl_auto_mode(self, b=None):
        '''
        开启与关闭自动模式
        :param b:
        :return:
        '''
        if b is not None:
            self.auto_mode = b

            if self.auto_mode:
                self.auto_nav_er.start()
            else:
                self.auto_nav_er.stop()
                self.car_op.move_stop()

        return self.auto_mode

    def handle_motion_system(self, action=''):
        if self.auto_mode:
            print('please close auto mode to use handle mode')
            return

        if action == 'forward':
            self.car_op.move_forward()
        elif action == 'back':
            self.car_op.move_back()
        elif action == 'left':
            self.car_op.move_left()
        elif action == 'right':
            self.car_op.move_right()
        elif action == 'stop':
            self.car_op.move_stop()

    def handle_servohead(self, action=''):
        if self.auto_mode:
            print('please close auto mode to use handle mode')
            return

        if action == 'turn_left':
            self.car_op.ctrl_servohead('turn_left')
        elif action == 'turn_right':
            self.car_op.ctrl_servohead('turn_right')
        elif action == 'turn_up':
            self.car_op.ctrl_servohead('turn_up')
        elif action == 'turn_down':
            self.car_op.ctrl_servohead('turn_down')

        elif action == 'watch_forward':
            self.car_op.ctrl_servohead('forward')
        elif action == 'watch_left':
            self.car_op.ctrl_servohead('left')
        elif action == 'watch_right':
            self.car_op.ctrl_servohead('right')
        elif action == 'watch_top':
            self.car_op.ctrl_servohead('top')

    def run(self):
        while True:
            time.sleep(1.)

            if not self.auto_mode:
                print('handling...')
                continue

            if self.car_op.offline:
                print('offline...')
                continue

            # 自动控制
            cmd_id = self.what_i_need_do()
            if cmd_id == 'idle':
                print('idle...')
            elif cmd_id == 'patrol':
                print('patrol...')
                self.auto_nav_er.start()
                self.auto_nav_er.to('any')
            elif cmd_id == 'stop':
                print('stop...')
                self.auto_nav_er.to('neighbor')
            else:
                raise AttributeError('Please check cmd code !')

    def get_current_img(self, only_left=True):
        '''
        返回当前图像，如果没有初始化成功，返回None
        :return: np.array
        '''
        img = self.car_op.get_cam_img()
        if only_left:
            img = np.split(img, 2, -1)[0]
        return img

    def get_current_jpg(self, only_left=True):
        '''
        返回当前图像的JPEG编码格式的字节串，如果没有初始化成功，返回None
        :return: None or bytes
        '''
        img = self.get_current_img(only_left)
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            _, data = cv2.imencode('.jpg', img, (cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality))
            return bytes(data)
        return None


if __name__ == '__main__':
    car = SmartCar()
    car.run()
