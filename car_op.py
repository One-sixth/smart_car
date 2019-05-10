import numpy as np
import cv2
import zmq
import time
import threading
from collections import deque
import car_op_cmd
from copy import deepcopy


class CarOp:
    target_addr = '192.168.137.241'
    is_quit = False
    img = None
    delay = 0.1

    offline = True

    move_status = 0
    move_status_last_setting_time = 0

    wait_to_send = deque(maxlen=500)
    recv_msg_quene = deque(maxlen=5000)

    current_msg_id = 1

    # 超声雷达状态：左中右，False代表前进空间小于12cm，True代表大于等于12cm
    ultrasonic_radar_status = [False, False, False]

    def __init__(self):
        self.ctx = zmq.Context(4)   # zmq 上下文
        self.cam_img_recv_socket = self.ctx.socket(zmq.PULL)          # 用于接收图像信息
        self.ctrl_send_socket = self.ctx.socket(zmq.PUSH)             # 用于发送控制信息
        self.msg_recv_socket = self.ctx.socket(zmq.PULL)              # 用于接收消息

        # 接收信息的端口全部设定0.5s等待
        self.cam_img_recv_socket.setsockopt(zmq.RCVTIMEO, 500)        # 设置控制0.5s超时
        self.msg_recv_socket.setsockopt(zmq.RCVTIMEO, 500)            # 设置控制0.5s超时

        self.cam_img_recv_socket.setsockopt(zmq.LINGER, 0)            # 设定关闭端口时立即清空队列，不设定此项将会卡住
        self.ctrl_send_socket.setsockopt(zmq.LINGER, 0)
        self.msg_recv_socket.setsockopt(zmq.LINGER, 0)

        self.cam_img_recv_socket.connect('tcp://%s:35687' % self.target_addr)
        self.ctrl_send_socket.connect('tcp://%s:35688' % self.target_addr)
        self.msg_recv_socket.connect('tcp://%s:35689' % self.target_addr)

        # 启动工作线程
        self.ctrl_send_thread = threading.Thread(target=self._ctrl_send_run)
        self.msg_recv_thread = threading.Thread(target=self._msg_recv_run)
        self.viewer_thread = threading.Thread(target=self._cam_img_recv_run)
        # self.reset_key_thread = threading.Thread(target=self._reset_key_run)

        self.ctrl_send_thread.start()
        self.msg_recv_thread.start()
        self.viewer_thread.start()
        # self.reset_key_thread.start()

    def _send_msg(self, msg):
        # 发送消息后，返回消息id，需凭借消息id接收消息
        msg_id = self.current_msg_id
        self.current_msg_id += 1
        data = [msg_id]
        data.extend(msg)
        self.wait_to_send.append(data)
        return msg_id

    def _recv_msg(self, msg_id, timeout=2.):
        # 获取消息后，将id和值设定为None，代表清除消息
        # 超时后，返回None
        msg = None
        for _ in range(int(timeout * 100)):
            time.sleep(0.01)
            for it in list(self.recv_msg_quene)[::-1]:
                if it[0] == msg_id:
                    msg = deepcopy(it[1:])
                    it[0] = None
                    break
            if msg is not None:
                break
        return msg

    def _ctrl_send_run(self):
        while not self.is_quit:
            if len(self.wait_to_send) > 0:
                msg = self.wait_to_send.pop()
            else:
                msg = [0, self.move_status]
            try:
                self.ctrl_send_socket.send_pyobj(msg, zmq.NOBLOCK)
                self.offline = False
            except zmq.error.Again:
                self.offline = True
            time.sleep(0.02)

    def _msg_recv_run(self):
        while not self.is_quit:
            try:
                msg = self.msg_recv_socket.recv_pyobj()
                self.recv_msg_quene.append(msg)
            except zmq.error.Again:
                pass

    def _cam_img_recv_run(self):
        while not self.is_quit:
            try:
                msg = self.cam_img_recv_socket.recv_pyobj()
                img_data, radar_status = msg
                self.ultrasonic_radar_status = radar_status
                self.img = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)
            except zmq.error.Again:
                pass

    def get_cam_img(self):
        return self.img

    def ctrl_standby(self, is_standby: bool):
        self._send_msg([car_op_cmd.msg_set_cam_standby, is_standby])

    def ctrl_fps(self, fps=None):
        if fps is None:
            msg = [car_op_cmd.msg_get_fps]
            msg_id = self._send_msg(msg)
            return self._recv_msg(msg_id)
        else:
            msg = [car_op_cmd.msg_set_fps, fps]
            self._send_msg(msg)

    def ctrl_jpeg_quality(self, jpeg_quality=None):
        if jpeg_quality is None:
            msg = [car_op_cmd.msg_get_jpeg_quality]
            msg_id = self._send_msg(msg)
            jpeg_quality = self._recv_msg(msg_id)
            return jpeg_quality
        else:
            msg = [car_op_cmd.msg_set_jpeg_quality, jpeg_quality]
            self._send_msg(msg)

    def ctrl_servohead(self, angle=None):
        if angle is None:
            msg = [car_op_cmd.msg_servo_head_get_angle]
            msg_id = self._send_msg(msg)
            angle = self._recv_msg(msg_id)
            return angle
        elif angle == 'forward':
            msg = [car_op_cmd.msg_servo_head_watch_forward]
        elif angle == 'left':
            msg = [car_op_cmd.msg_servo_head_watch_left]
        elif angle == 'right':
            msg = [car_op_cmd.msg_servo_head_watch_right]
        elif angle == 'top':
            msg = [car_op_cmd.msg_servo_head_watch_top]
        elif angle == 'horizontal':
            msg = [car_op_cmd.msg_servo_head_watch_horizontal]
        elif angle == 'turn_up':
            msg = [car_op_cmd.msg_servo_head_turn_up]
        elif angle == 'turn_down':
            msg = [car_op_cmd.msg_servo_head_turn_down]
        elif angle == 'turn_left':
            msg = [car_op_cmd.msg_servo_head_turn_left]
        elif angle == 'turn_right':
            msg = [car_op_cmd.msg_servo_head_turn_right]
        else:
            msg = [car_op_cmd.msg_servo_head_set_angle, *angle]
        self._send_msg(msg)

    def move_forward(self):
        self.move_status = car_op_cmd.move_forward

    def move_back(self):
        self.move_status = car_op_cmd.move_back

    def move_left(self):
        self.move_status = car_op_cmd.move_left

    def move_right(self):
        self.move_status = car_op_cmd.move_right

    def move_stop(self):
        self.move_status = car_op_cmd.move_stop

    def Cleanup(self):
        self.is_quit = True
        self.viewer_thread.join()
        self.msg_recv_thread.join()
        # self.reset_key_thread.join()
        self.ctrl_send_thread.join()
        time.sleep(0.3)
        cv2.destroyAllWindows()
        self.cam_img_recv_socket.close()
        self.ctrl_send_socket.close()
        self.msg_recv_socket.close()
        self.ctx.destroy()
        print('Cleanup finish')


if __name__ == '__main__':
    op = CarOp()

    while True:
        key = cv2.waitKey(1000 // 30)
        img = op.get_cam_img()
        if img is None:
            continue
        img = cv2.cvtColor(op.get_cam_img(), cv2.COLOR_RGB2BGR)
        cv2.imshow('viewer', img)
        time.sleep(0.01)
        last_time = time.time()

        if key == ord('w'):
            op.move_forward()
        elif key == ord('s'):
            op.move_back()
        elif key == ord('a'):
            op.move_left()
        elif key == ord('d'):
            op.move_right()
        elif key == ord(' '):
            op.move_stop()

        elif key == ord('r'):
            op.ctrl_jpeg_quality(20)
        elif key == ord('t'):
            op.ctrl_jpeg_quality(80)
        elif key == ord('y'):
            print(op.ctrl_jpeg_quality())

        elif key == ord('f'):
            op.ctrl_fps(60)
        elif key == ord('g'):
            op.ctrl_fps(15)
        elif key == ord('h'):
            print(op.ctrl_fps())

        elif key == ord('v'):
            op.ctrl_standby(True)
        elif key == ord('b'):
            op.ctrl_standby(False)

        elif key == ord('7'):
            op.ctrl_servohead('top')
        elif key == ord('9'):
            op.ctrl_servohead('horizontal')
        elif key == ord('1'):
            op.ctrl_servohead('left')
        elif key == ord('3'):
            op.ctrl_servohead('right')
        elif key == ord('5'):
            op.ctrl_servohead('forward')
        elif key == ord('-'):
            op.ctrl_servohead([8, None])
        elif key == ord('+'):
            op.ctrl_servohead([16, None])

        elif key == ord('2'):
            op.ctrl_servohead('turn_down')
        elif key == ord('8'):
            op.ctrl_servohead('turn_up')
        elif key == ord('4'):
            op.ctrl_servohead('turn_left')
        elif key == ord('6'):
            op.ctrl_servohead('turn_right')
        elif key == ord('1'):
            print(op.ctrl_servohead())

        elif key == ord('l'):
            print(op.ultrasonic_radar_status)

        elif key == ord('t'):
            op.Cleanup()
            break
