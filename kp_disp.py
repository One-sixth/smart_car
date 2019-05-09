import cv2
import numpy as np
import imageio
import math


class LocalStereoKeypointLib():
    def __init__(self, cam_l_dist, cam_r_dist, cam_t, cam_r, keypoint_type='surf'):
        if keypoint_type.lower() == 'surf':
            self._detect = cv2.xfeatures2d.SURF_create()
        elif keypoint_type.lower() == 'orb':
            self._detect = cv2.ORB_create()
        elif keypoint_type.lower() == 'akaze':
            self._detect = cv2.AKAZE_create()
        elif keypoint_type.lower() == 'sift':
            self._detect = cv2.SIFT_create()

    def calc_pointcloud(self, img_l, img_r):
        assert img_l.shape == img_r.shape, 'img_l.shape must be equal img_r.shape'
        kps_l, desc_l = self._detect.detectAndCompute(img_l, None)
        kps_r, desc_r = self._detect.detectAndCompute(img_r, None)

        desc_l = np.asarray(desc_l, np.float32)
        desc_r = np.asarray(desc_r, np.float32)

        # 排除错误匹配点
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches1 = flann.knnMatch(desc_l, desc_r, k=2)

        good_matches1 = []
        for m, n in matches1:
            if m.distance < 0.7 * n.distance:
                # if math.fabs(kps1[m.queryIdx].pt[1] - kps2[m.trainIdx].pt[1]) < 20 and math.fabs(kps1[m.queryIdx].pt[0] - kps2[m.trainIdx].pt[0]) < 60:
                # 因为是双目摄像头，视差有一定的限制
                good_matches1.append(m)

        good_matches2 = cv2.xfeatures2d.matchGMS((img_l.shape[:2][::-1]), (img_r.shape[:2][::-1]), kps_l, kps_r,
                                                 good_matches1)

        good_kps_l = [kps_l[m.queryIdx] for m in good_matches2]
        good_kps_r = [kps_r[m.trainIdx] for m in good_matches2]

        im = cv2.drawMatches(img_l, kps_l, img_r, kps_r, good_matches2, None)
        im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        cv2.imshow('show', im)

        disp_img = np.zeros_like(img_l)

        # 稀疏点视差图
        for kp_l, kp_r in zip(good_kps_l, good_kps_r):
            pos = np.int32(np.round(kp_l.pt))
            distance = np.float32(kp_l.pt)[1] - np.float32(kp_r.pt)[1]
            disp_img[pos[1], pos[0]] = -distance * 30

        cv2.imshow('disp_pt', disp_img)

        cv2.waitKey(0)


if __name__ == '__main__':
    # # cam1 = imageio.get_reader('<video0>', size=(640, 480), fps=30)
    c = LocalStereoKeypointLib(None, None, None, None)
    img_l = imageio.imread('1.png')
    img_r = imageio.imread('2.png')

    c.calc_pointcloud(img_l, img_r)
    # 
    # kps1, desc1 = surf.detectAndCompute(img1, None)
    # kps2, desc2 = surf.detectAndCompute(img2, None)
    # 
    # desc1 = np.asarray(desc1, np.float32)
    # desc2 = np.asarray(desc2, np.float32)
    # 
    # FLANN_INDEX_KDTREE = 1
    # index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    # search_params = dict(checks=50)
    # flann = cv2.FlannBasedMatcher(index_params, search_params)
    # matches1 = flann.knnMatch(desc1, desc2, k=2)
    # 
    # good_matches1 = []
    # for m, n in matches1:
    #     if m.distance < 0.7*n.distance:
    #         # if math.fabs(kps1[m.queryIdx].pt[1] - kps2[m.trainIdx].pt[1]) < 20 and math.fabs(kps1[m.queryIdx].pt[0] - kps2[m.trainIdx].pt[0]) < 60:
    #             # 因为是双目摄像头，视差有一定的限制
    #             good_matches1.append(m)
    # 
    # good_matches2 = cv2.xfeatures2d.matchGMS((img1.shape[:2][::-1]), (img2.shape[:2][::-1]), kps1, kps2, good_matches1)
    # 
    # good_kps1 = [kps1[m.queryIdx] for m in good_matches2]
    # good_kps2 = [kps2[m.trainIdx] for m in good_matches2]
    # 
    # im = cv2.drawMatches(img1, kps1, img2, kps2, good_matches2, None)
    # im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    # cv2.imshow('show', im)
    # 
    # disp_img = np.zeros_like(img1)
    # 
    # # 稀疏点视差图
    # for m in good_matches2:
    #     pos = np.int32(np.round(kps1[m.queryIdx].pt))
    #     distance = np.linalg.norm(np.float32(kps1[m.queryIdx].pt) - np.float32(kps2[m.trainIdx].pt))
    #     disp_img[pos[1], pos[0]] = distance
    # 
    # cv2.imshow('disp_pt', disp_img)
    # 
    # cv2.waitKey(0)