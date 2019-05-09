import zmq
import cv2
import numpy as np
import time
import imageio


class OrbSlam2Connector:
    # slam server 地址
    slam_server_address = '127.0.0.1:35696'
    # 掉线标记
    offline = True
    # 连接时最大尝试次数
    max_try_count = 10


    _SLAM_SYSTEM_NOT_READY = -1,
    _SLAM_NO_IMAGES_YET = 0,
    _SLAM_NOT_INITIALIZED = 1,
    _SLAM_OK = 2,
    _SLAM_LOST = 3

    def __init__(self):
        self.start_time = time.time()
        print('orbslam2_connector: init slam connect')
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        # 要求如果没有建立链接，立刻返回
        self.socket.setsockopt(zmq.IMMEDIATE, True)
        # 要求应答序号要一一对应
        self.socket.setsockopt(zmq.REQ_CORRELATE, True)
        # 要求不严格轮换
        self.socket.setsockopt(zmq.REQ_RELAXED, True)

        self.socket.connect('tcp://%s' % self.slam_server_address)
        while self.offline:
            try:
                print('orbslam2_connector: try say hello to slam server')
                self.socket.send(bytes(np.array([-1], np.int32)))
                data = self.socket.recv()
                if len(data) == 4:
                    data = int(np.frombuffer(data, np.int))
                    if data == -1:
                        print('orbslam2_connector: connect success')
                        self.offline = False
                        continue
                print('wrong data')
            except zmq.error.Again:
                print('orbslam2_connector: connect out of time, will try again')


    def get_pos(self, imgLR):
        assert imgLR.shape == (360, 1280, 3)

        ts = time.time() - self.start_time
        ts = int(ts*1000)
        mtype = bytes(np.array([1], np.int32))
        timestamp = bytes(np.array([ts], np.int32))
        imcode = bytes(cv2.imencode('.jpg', imgLR)[1])
        imcode_len = bytes(np.array([len(imcode)], np.uint32))
        msg = mtype + timestamp + imcode_len + imcode

        try:
            self.socket.send(msg, copy=False)
            self.offline = False
        except zmq.error.Again:
            self.offline = True

        # 如果没有成功发送
        if self.offline:
            print('orbslam2_connector: found offline when send data')
            return False, None

        try:
            data = self.socket.recv()
            self.offline = False
        except zmq.error.Again:
            self.offline = True

        # 如果没有成功接收
        if self.offline:
            print('orbslam2_connector: found offline when recv data')
            return False, None

        if len(data) != 76:
            print('orbslam2_connector: recv data wrong')
            return False, None

        mtype = np.frombuffer(data[0:4], np.int32)[0]
        slam_status = np.frombuffer(data[4:8], np.int32)[0]
        rel_mat = np.frombuffer(data[8:72], np.float32).reshape([4, 4])
        restart_count = np.frombuffer(data[72:76], np.int32)[0]

        if mtype != 1:
            print('orbslam2_connector: recv data wrong')
            return False, None

        if slam_status != self._SLAM_OK:
            print('SLAM is not OK')
            return False, None

        return restart_count, rel_mat

    def get_restart_count(self):
        pass

    def get_status(self):
        pass


if __name__ == '__main__':
    osc = OrbSlam2Connector()
    cam1 = imageio.get_reader('<video1>', size=(1280, 720))
    cam2 = imageio.get_reader('<video2>', size=(1280, 720))

    # 读取相机参数
    stereo_cam_params_file = 'stereo_cam_params_imx322_1280x720.yml'
    f = cv2.FileStorage(stereo_cam_params_file, cv2.FILE_STORAGE_READ)
    matL = f.getNode('LM').mat()
    distL = f.getNode('LD').mat()
    matR = f.getNode('RM').mat()
    distR = f.getNode('RD').mat()
    R = f.getNode('R').mat()
    T = f.getNode('T').mat()
    hw = np.array([720, 1280])
    f.release()

    # 设定立体矩阵
    wh = tuple(hw[::-1])
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = \
        cv2.stereoRectify(matL, distL, matR, distR,
                          wh, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=wh)

    mapL1, mapL2 = cv2.initUndistortRectifyMap(matL, distL, R1, P1, wh, cv2.CV_16SC2)
    mapR1, mapR2 = cv2.initUndistortRectifyMap(matR, distR, R2, P2, wh, cv2.CV_16SC2)

    while True:
        imgL = cam1.get_next_data()
        imgR = cam2.get_next_data()

        imgL = cv2.rotate(imgL, cv2.ROTATE_180)
        imgR = cv2.rotate(imgR, cv2.ROTATE_180)

        # 校畸
        imgL = cv2.remap(imgL, mapL1, mapL2, cv2.INTER_LINEAR)
        imgR = cv2.remap(imgR, mapR1, mapR2, cv2.INTER_LINEAR)
        # imgL = cv2.resize(imgL, (640, 360), cv2.INTER_AREA)
        # imgR = cv2.resize(imgR, (640, 360), cv2.INTER_AREA)

        imgLR = np.concatenate([imgL, imgR], 1)
        imgLR = cv2.resize(imgLR, (1280, 360), interpolation=cv2.INTER_AREA)
        im_show = cv2.cvtColor(imgLR, cv2.COLOR_RGB2BGR)
        cv2.imshow('view', im_show)
        s, m = osc.get_pos(imgLR)
        print(s)
        cv2.waitKey(1000//30)

