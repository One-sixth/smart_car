import time
import cv2
import numpy as np
from AutoNav import AutoNav
from car_op import CarOp
from orbslam2_connector import OrbSlam2Connector
from threading import Thread
from Yolov3HumanDetector import Yolov3HumanDetector
from auto_watch_human import AutoWatchHuman
from Notifier import Notifier
import notify_type
from Recorder import Recorder


class SmartCar:
    # 自动模式
    auto_mode = True

    current_status = ''

    jpeg_quality = 80

    _human_boxes = []
    draw_human_boxes = True

    def __init__(self, *, debug_show=False):
        self.car_op = CarOp()
        self.auto_nav_er = AutoNav(self.car_op)
        # self.orbslam2_connect = OrbSlam2Connector()
        self.debug_show = debug_show

        self.human_detect = Yolov3HumanDetector()
        self.auto_watch_human = AutoWatchHuman(self.car_op)

        self.recorder = Recorder()
        self.notifier = Notifier(self.recorder)

        self.work_thread = Thread(target=self.work_run)
        self.work_thread.start()

        self.record_thread = Thread(target=self.record_run)
        self.record_thread.start()

    def work_run(self):
        while True:
            time.sleep(1/3)
            img = self.get_current_img(only_left=True)
            if img is None:
                continue

            self.human_detect.push_frame(img)
            boxes = self.human_detect.detect()
            if len(boxes) > 0:
                boxes = np.array(boxes, np.float32)
                self._human_boxes = boxes.copy()
                self.auto_watch_human.detect(img, boxes)
                self.notifier.notice(notify_type.type_human, '发现人')
                self.notifier.do_interest_mark()
            else:
                self._human_boxes = []

            if self.debug_show:
                img = self.get_current_img(only_left=True, draw_box=True)
                img2 = img[:, :, ::-1]
                cv2.imshow('viewerL', img2)
                cv2.waitKey(1000 // 60)

    def record_run(self):
        while True:
            time.sleep(1/20)
            img = self.get_current_img(only_left=True)
            if img is None:
                continue
            if self.notifier.can_record:
                self.recorder.start_video_record(20)
            else:
                self.recorder.stop_video_record()
            self.recorder.next_frame(img)

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

            # if imgLR is not None:
            #     self.orbslam2_connect.get_pos(imgLR)

            if self.car_op.offline:
                print('offline...')
                continue

            if not self.auto_mode:
                print('handling...')
                continue

            # 自动控制
            cmd_id = self.what_i_need_do()
            if self.current_status != cmd_id:
                self.current_status = cmd_id
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

    def get_current_img(self, only_left=True, draw_box=False):
        '''
        返回当前图像，如果没有初始化成功，返回None
        :return: np.array
        '''

        img = self.car_op.get_cam_img()
        if img is None:
            return None

        if only_left:
            img = np.split(img, 2, 1)[0]

        if draw_box:
            human_boxes = self._human_boxes.copy()
            for box in human_boxes:
                cv2.rectangle(img, tuple(box[:2]), tuple(box[2:]), (0, 255, 0))

        return img

    def get_current_jpg(self, only_left=True):
        '''
        返回当前图像的JPEG编码格式的字节串，如果没有初始化成功，返回None
        :return: None or bytes
        '''
        img = self.get_current_img(only_left, draw_box=self.draw_human_boxes)
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            _, data = cv2.imencode('.jpg', img, (cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality))
            return bytes(data)
        return None


if __name__ == '__main__':
    car = SmartCar(debug_show=True)
    car.ctrl_auto_mode(False)
    car.run()
