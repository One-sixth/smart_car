import cv2
import numpy as np


class StereoDepthMap:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    hw = None
    validPixROI1 = None
    validPixROI2 = None
    mapL1 = None
    mapL2 = None
    mapR1 = None
    mapR2 = None

    fx = None
    b = None

    def __init__(self, bm_type='bm'):
        self.bm_type = bm_type
        unitDisparity = 15
        numberOfDisaprities = unitDisparity * 16

        if bm_type == 'bm':
            bm = cv2.StereoBM().create()
            # bm.setROI1(validPixROI1)
            # bm.setROI2(validPixROI2)
            bm.setBlockSize(21)
            bm.setPreFilterCap(13)
            bm.setMinDisparity(0)
            bm.setNumDisparities(numberOfDisaprities)
            bm.setTextureThreshold(20)
            bm.setUniquenessRatio(10)
            bm.setSpeckleWindowSize(19)
            bm.setSpeckleRange(32)
            bm.setDisp12MaxDiff(5)
            self.bm = bm

        elif bm_type == 'sgbm':
            bm = cv2.StereoSGBM().create()
            SADWindowSize = 11
            mindisparity = 0
            ndisparities = 64
            bm.setMinDisparity(0)
            bm.setNumDisparities(64)
            bm.setBlockSize(SADWindowSize)
            channel = 1
            P1 = 8 * channel * SADWindowSize * SADWindowSize
            P2 = 32 * channel * SADWindowSize * SADWindowSize
            bm.setP1(P1)
            bm.setP2(P2)
            bm.setPreFilterCap(15)
            bm.setUniquenessRatio(10)
            bm.setSpeckleRange(2)
            bm.setSpeckleWindowSize(100)
            bm.setDisp12MaxDiff(cv2.STEREO_SGBM_MODE_SGBM)
            self.bm = bm
        else:
            raise AssertionError('bm_type must be select in sgbm and bm')

    def init_with_map_matrix(self, hw, mapL1, mapL2, mapR1, mapR2):
        self.hw = hw
        self.mapL1 = mapL1
        self.mapL2 = mapL2
        self.mapR1 = mapR1
        self.mapR2 = mapR2

    def init_with_cfg(self, hw, matL, distL, matR, distR, R, T):
        wh = tuple(hw[::-1])
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = \
            cv2.stereoRectify(matL, distL, matR, distR,
                              wh, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=wh)

        mapL1, mapL2 = cv2.initUndistortRectifyMap(matL, distL, R1, P1, wh, cv2.CV_16SC2)
        mapR1, mapR2 = cv2.initUndistortRectifyMap(matR, distR, R2, P2, wh, cv2.CV_16SC2)

        self.hw = hw
        self.validPixROI1 = validPixROI1
        self.validPixROI2 = validPixROI2
        self.mapL1 = mapL1
        self.mapL2 = mapL2
        self.mapR1 = mapR1
        self.mapR2 = mapR2

        self.fx = matL[0, 0]
        self.b = np.linalg.norm(T, 2)

    def next_img(self, imgL, imgR):
        if self.bm_type == 'bm':
            imgL = cv2.cvtColor(imgL, cv2.COLOR_RGB2GRAY)
            imgR = cv2.cvtColor(imgR, cv2.COLOR_RGB2GRAY)

        imgL = cv2.remap(imgL, self.mapL1, self.mapL2, cv2.INTER_LINEAR)
        imgR = cv2.remap(imgR, self.mapR1, self.mapR2, cv2.INTER_LINEAR)

        imgL = cv2.resize(imgL, (640, 360), cv2.INTER_AREA)
        imgR = cv2.resize(imgR, (640, 360), cv2.INTER_AREA)

        disp = self.bm.compute(imgL, imgR)
        disp = disp / 16. / 255.

        disp_v = F_Gray2Color(disp.clip(0, 1))

        cv2.imshow('disp', cv2.cvtColor(disp_v, cv2.COLOR_RGB2BGR))

        depth = disp * self.fx / self.b
        depth_v = F_Gray2Color((depth / 255).clip(0, 1))

        cv2.imshow('depth', cv2.cvtColor(depth_v, cv2.COLOR_RGB2BGR))

        return depth


def F_Gray2Color(vdisp):
    layer_r = vdisp
    layer_b = 1. - vdisp
    layer_g = 1. - np.abs(vdisp - (1. / 2)) * 2
    layer_rgb = np.dstack([layer_r, layer_g, layer_b])
    layer_rgb = layer_rgb * 255
    layer_rgb = np.asarray(layer_rgb, np.uint8)
    layer_rgb = np.where(np.dstack([vdisp]*3) <= 0, np.zeros_like(layer_rgb), layer_rgb)
    return layer_rgb


if __name__ == '__main__':
    import glob
    import os
    import imageio

    dm = StereoDepthMap('sgbm')

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

    dm.init_with_cfg(hw, matL, distL, matR, distR, R, T)

    cam1 = imageio.get_reader('<video1>', size=tuple(hw[::-1]), fps=30)
    cam2 = imageio.get_reader('<video2>', size=tuple(hw[::-1]), fps=30)

    while True:
        img1 = cam1.get_next_data()
        img2 = cam2.get_next_data()
        cv2.imshow('a', img1[..., ::-1])
        cv2.imshow('b', img2[..., ::-1])
        dm.next_img(img1, img2)
        cv2.waitKey(16)
