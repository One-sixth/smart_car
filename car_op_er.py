import numpy as np
import cv2
import zmq
import time
import threading
from collections import deque
import imageio
import car_op_cmd
try:
    from MotionSystem import MotionSystem
    _no_motion_system = False
    from ServoHead import ServoHead
    _no_servo_head = False
    from UltrasonicRadar import UltrasonicRadar
    _no_ultrasonic_radar = False
except BaseException as e:
    print('Error, motion_system has been disable. error info {}'.format(e))
    _no_motion_system = True
    _no_servo_head = True
    _no_ultrasonic_radar = True

_allow_no_cam = True


class CarOpEr:
    is_quit = False

    # cam1_id = cv2.CAP_DSHOW + 1
    # cam2_id = cv2.CAP_DSHOW + 2

    cam1_id = 0
    cam2_id = 2

    frame_hw = [480, 640]

    log_quene = deque(maxlen=5000)
    wait_to_send = deque(maxlen=100)
    recv_msg_quene = deque(maxlen=5000)
    jpeg_quality = 80
    fps = 30

    # 超声雷达状态：左中右，False代表前进空间小于12cm，True代表大于等于12cm
    ultrasonic_radar_status = [False, False, False]

    def __init__(self):
        self.log_quene.append('CarOpEr：start init')

        if not _no_motion_system:
            self.motion_system = MotionSystem()
            self.log_quene.append('CarOpEr：found motion_system system')

        if not _no_servo_head:
            self.servo_head = ServoHead()
            self.log_quene.append('CarOpEr：found servo_head system')

        if not _no_ultrasonic_radar:
            self.ultrasonic_radar = UltrasonicRadar()
            self.log_quene.append('CarOpEr：found ultrasonic_radar system')

        self.cam1 = cv2.VideoCapture(self.cam1_id)
        self.cam2 = cv2.VideoCapture(self.cam2_id)

        self.cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_hw[0])
        self.cam1.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_hw[1])
        self.cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_hw[0])
        self.cam2.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_hw[1])

        if not(self.cam1.isOpened() and self.cam2.isOpened()):
            msg = 'use cam failure, cam1 status: %d ; cam2 status: %d' % (self.cam1.isOpened(), self.cam2.isOpened())
            print(msg)
            self.log_quene.append('CarOpEr：%s' % msg)
            self.cam1 = None
            self.cam2 = None
            self.no_cam = True
        else:
            msg = 'use cam success'
            print(msg)
            self.log_quene.append('CarOpEr：%s' % msg)
            self.no_cam = False

        if self.no_cam and not _allow_no_cam:
            raise AttributeError('No cam!!!')

        self.cam_img = np.zeros([self.frame_hw[0], self.frame_hw[1]*2, 1], np.uint8)
        # self.cam_img = cv2.randu(self.cam_img, 10, 250)

        self.ctx = zmq.Context(4)
        self.cam_img_send_socket = self.ctx.socket(zmq.PUSH)    # 用于发送图像信息
        self.ctrl_recv_socket = self.ctx.socket(zmq.PULL)       # 用于接收控制信息
        self.msg_send_socket = self.ctx.socket(zmq.PUSH)        # 用于发送消息

        self.cam_img_send_socket.setsockopt(zmq.LINGER, 0)  # 设定关闭端口时立即清空队列，不设定此项将会卡住
        self.ctrl_recv_socket.setsockopt(zmq.LINGER, 0)
        self.msg_send_socket.setsockopt(zmq.LINGER, 0)

        # 图像发送端口重要的是实时性，过时的消息不要发送，而是直接丢掉
        # # 设定图像发送端口只缓存最多2条信息，系统发送缓冲区最大为1MB
        # self.cam_img_send_socket.setsockopt(zmq.SNDHWM, 2)
        # self.cam_img_send_socket.setsockopt(zmq.SNDBUF, 1024*512*1)
        # 这个可以替代以上功能，只保留消息队列中最后一个消息，其余消息会被丢弃
        self.cam_img_send_socket.setsockopt(zmq.CONFLATE, True)
        # 这个可以保证在连接断开或没有建立时，消息会被直接丢弃，而不是在消息队列里面排队
        self.cam_img_send_socket.setsockopt(zmq.IMMEDIATE, True)

        self.ctrl_recv_socket.setsockopt(zmq.RCVTIMEO, 100)   # 设置控制1s超时

        self.cam_img_send_socket.bind('tcp://*:35687')
        self.ctrl_recv_socket.bind('tcp://*:35688')
        self.msg_send_socket.bind('tcp://*:35689')

        self.move_status = 0
        self.move_status_last_setting_time = 0

        self.cam_standby = False    # 相机待机
        self.has_standby = False    # 相机是否已经待机？

        self.msg_send_thread = threading.Thread(target=self.msg_send_run)
        self.msg_send_thread.start()

        if not self.no_cam:
            self.capture_thread = threading.Thread(target=self.capture_run)
            self.cam_img_send_thread = threading.Thread(target=self.cam_img_send_run)

            self.capture_thread.start()
            self.cam_img_send_thread.start()

    def set_framehw(self, framehw):
        if framehw == self.frame_hw:
            return
        self.cam_standby = True
        while not self.has_standby:
            time.sleep(0.1)
        self.frame_hw = framehw
        self.cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_hw[0])
        self.cam1.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_hw[1])
        self.cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_hw[0])
        self.cam2.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_hw[1])
        self.cam_standby = False

    def send_msg(self, msg_id, msg):
        # 发送消息，与op的相反，这里需要op发来消息的id
        data = [msg_id]
        data.extend(msg)
        self.wait_to_send.append(data)
        return msg_id

    def recv_msg(self):
        # 获取消息后，将id和值设定为None，代表清除消息
        # 超时后，返回None
        msg = None
        try:
            msg = self.ctrl_recv_socket.recv_pyobj()
        except zmq.error.Again:
            pass
        return msg

    def capture_run(self):
        # 超声雷达状态通过相机线程刷新
        while not self.is_quit:

            if not _no_ultrasonic_radar:
                l = self.ultrasonic_radar.check_left()
                m = self.ultrasonic_radar.check_forward()
                r = self.ultrasonic_radar.check_right()
                self.ultrasonic_radar_status = [l, m, r]
            else:
                self.ultrasonic_radar_status = [True, True, True]

            if self.cam_standby:
                self.has_standby = True
                time.sleep(1.)
            else:
                self.has_standby = False
                if self.cam1.grab() and self.cam2.grab():
                    _, img1 = self.cam1.retrieve()
                    _, img2 = self.cam2.retrieve()
                    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
                    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

                    img1 = cv2.rotate(img1, cv2.ROTATE_180)
                    img2 = cv2.rotate(img2, cv2.ROTATE_180)

                    self.cam_img = np.concatenate([img1, img2], 1)
                    # self.cam_img = cv2.resize(self.cam_img, (self.cam_img.shape[0]//2, self.cam_img.shape[1]//2), interpolation=cv2.INTER_AREA)
                else:
                    msg = 'grab failure'
                    print(msg)
                    self.log_quene.append('CarOpEr：%s' % msg)
            time.sleep(1/self.fps)

    def cam_img_send_run(self):
        while not self.is_quit:
            if self.cam_standby:
                time.sleep(1.)
            else:
                _, data = cv2.imencode('.jpg', self.cam_img, (cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality))
                self.cam_img_send_socket.send_pyobj([data, self.ultrasonic_radar_status])
            time.sleep(1/self.fps)

    def msg_send_run(self):
        while not self.is_quit:
            if len(self.wait_to_send) > 0:
                self.msg_send_socket.send_pyobj(self.wait_to_send.pop())
            time.sleep(1/self.fps)

    def run(self):
        # i = 0
        while not self.is_quit:
            msg = self.recv_msg()
            if msg is None:
                print('no msg')
                self.motion_system.stop()
                continue
            msg_id = msg[0]
            msg_data = msg[1:]
            # i+=1

            if msg_data[0] == car_op_cmd.move_stop and not _no_motion_system:
                self.motion_system.stop()
            elif msg_data[0] == car_op_cmd.move_forward and not _no_motion_system:
                self.motion_system.forward()
            elif msg_data[0] == car_op_cmd.move_back and not _no_motion_system:
                self.motion_system.back()
            elif msg_data[0] == car_op_cmd.move_left and not _no_motion_system:
                self.motion_system.left()
            elif msg_data[0] == car_op_cmd.move_right and not _no_motion_system:
                self.motion_system.right()

            # cmd get
            elif msg_data[0] == car_op_cmd.msg_get_fps:
                self.send_msg(msg_id, [self.fps])
            elif msg_data[0] == car_op_cmd.msg_get_jpeg_quality:
                self.send_msg(msg_id, [self.jpeg_quality])
            elif msg_data[0] == car_op_cmd.msg_get_framehw:
                self.send_msg(msg_id, [self.frame_hw[0], self.frame_hw[1]])
            elif msg_data[0] == car_op_cmd.msg_get_log:
                self.send_msg(msg_id, [list(self.log_quene)])
                self.log_quene.clear()

            # cmd set
            elif msg_data[0] == car_op_cmd.msg_set_fps:
                self.fps = int(msg_data[1])
            elif msg_data[0] == car_op_cmd.msg_set_jpeg_quality:
                self.jpeg_quality = int(msg_data[1])
            elif msg_data[0] == car_op_cmd.msg_set_framehw:
                self.set_framehw(msg_data[1])
            elif msg_data[0] == car_op_cmd.msg_set_cam_standby:
                self.cam_standby = bool(msg_data[1])

            # cmd servohead control
            elif msg_data[0] == car_op_cmd.msg_servo_head_get_angle and not _no_servo_head:
                self.send_msg(msg_id, [*self.servo_head.get_angle()])
            elif msg_data[0] == car_op_cmd.msg_servo_head_set_angle and not _no_servo_head:
                self.servo_head.watch_angle(*msg_data[1:])
            elif msg_data[0] == car_op_cmd.msg_servo_head_watch_forward and not _no_servo_head:
                self.servo_head.watch_forward()
            elif msg_data[0] == car_op_cmd.msg_servo_head_watch_left and not _no_servo_head:
                self.servo_head.watch_left()
            elif msg_data[0] == car_op_cmd.msg_servo_head_watch_right and not _no_servo_head:
                self.servo_head.watch_right()
            elif msg_data[0] == car_op_cmd.msg_servo_head_watch_top and not _no_servo_head:
                self.servo_head.watch_top()
            elif msg_data[0] == car_op_cmd.msg_servo_head_watch_horizontal and not _no_servo_head:
                self.servo_head.watch_horizontal()

            elif msg_data[0] == car_op_cmd.msg_servo_head_turn_up and not _no_servo_head:
                self.servo_head.turn_up()
            elif msg_data[0] == car_op_cmd.msg_servo_head_turn_down and not _no_servo_head:
                self.servo_head.turn_down()
            elif msg_data[0] == car_op_cmd.msg_servo_head_turn_left and not _no_servo_head:
                self.servo_head.turn_left()
            elif msg_data[0] == car_op_cmd.msg_servo_head_turn_right and not _no_servo_head:
                self.servo_head.turn_right()

            print(msg_id, msg_data)


if __name__ == '__main__':
    oper = CarOpEr()
    oper.run()
