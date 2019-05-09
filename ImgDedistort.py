'''
双目图像去畸变类
'''

import cv2
import numpy as np


class StereoImgDedistort:
    def __init__(self, cfg_path='stereo_cam_params_imx322_1280x720.yml', *, debug_show=False):
        self.debug_show = debug_show
        f = cv2.FileStorage(cfg_path, cv2.FILE_STORAGE_READ)

        matL = f.getNode('LM').mat()
        distL = f.getNode('LD').mat()
        matR = f.getNode('RM').mat()
        distR = f.getNode('RD').mat()
        R = f.getNode('R').mat()
        T = f.getNode('T').mat()
        hw = f.getNode('HW').mat().squeeze()
        f.release()

        wh = tuple(hw[::-1])
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = \
            cv2.stereoRectify(matL, distL, matR, distR,
                              wh, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=wh)

        mapL1, mapL2 = cv2.initUndistortRectifyMap(matL, distL, R1, P1, wh, cv2.CV_16SC2)
        mapR1, mapR2 = cv2.initUndistortRectifyMap(matR, distR, R2, P2, wh, cv2.CV_16SC2)

        self.matL = matL
        self.matR = matR
        self.distL = distL
        self.distR = distR
        self.R = R
        self.T = T

        self.hw = hw
        self.validPixROI1 = validPixROI1
        self.validPixROI2 = validPixROI2
        self.mapL1 = mapL1
        self.mapL2 = mapL2
        self.mapR1 = mapR1
        self.mapR2 = mapR2

    def next_img(self, imgL, imgR):
        assert (imgL.shape[:2] == imgR.shape[:2] == self.hw).all(), 'input img hw wrong'

        imgL = cv2.remap(imgL, self.mapL1, self.mapL2, cv2.INTER_LINEAR)
        imgR = cv2.remap(imgR, self.mapR1, self.mapR2, cv2.INTER_LINEAR)

        imgL = cv2.resize(imgL, (640, 360), cv2.INTER_AREA)
        imgR = cv2.resize(imgR, (640, 360), cv2.INTER_AREA)

        if self.debug_show:
            imgLR = np.concatenate([imgL, imgR], 1)
            cv2.imshow('stereo_undistort', cv2.cvtColor(imgLR, cv2.COLOR_RGB2BGR))
        return imgL, imgR


if __name__ == '__main__':
    import imageio

    scd = StereoImgDedistort(debug_show=True)

    cam1 = imageio.get_reader('<video1>', size=(1280, 720), fps=30)
    cam2 = imageio.get_reader('<video2>', size=(1280, 720), fps=30)

    while True:
        img1 = cam1.get_next_data()
        img2 = cam2.get_next_data()
        scd.next_img(img1, img2)
        cv2.waitKey(1000//60)
