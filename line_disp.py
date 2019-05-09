import cv2
import numpy as np
import imageio
import math


# cam1 = imageio.get_reader('<video0>', size=(640, 480), fps=30)
img1 = imageio.imread('1.png')
img2 = imageio.imread('2.png')

lsd = cv2.line_descriptor_LSDDetector.createLSDDetector()
# surf = cv2.ORB_create(1000)
# surf = cv2.xfeatures2d.VGG_create()
# surf = cv2.AKAZE_create()

desc1 = lsd.detect(img1, 1, 1)
desc2 = lsd.detect(img2, 1, 1)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches1 = flann.knnMatch(desc1, desc2, k=2)

good_matches1 = []
for m, n in matches1:
    if m.distance < 0.7*n.distance:
        # if math.fabs(kps1[m.queryIdx].pt[1] - kps2[m.trainIdx].pt[1]) < 20 and math.fabs(kps1[m.queryIdx].pt[0] - kps2[m.trainIdx].pt[0]) < 60:
            # 因为是双目摄像头，视差有一定的限制
            good_matches1.append(m)

good_matches2 = cv2.xfeatures2d.matchGMS((img1.shape[:2][::-1]), (img2.shape[:2][::-1]), kps1, kps2, good_matches1)

good_kps1 = [kps1[m.queryIdx] for m in good_matches2]
good_kps2 = [kps2[m.trainIdx] for m in good_matches2]

im = cv2.drawMatches(img1, kps1, img2, kps2, good_matches2, None)
im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
cv2.imshow('show', im)

disp_img = np.zeros_like(img1)

# 稀疏点视差图
for m in good_matches2:
    pos = np.int32(np.round(kps1[m.queryIdx].pt))
    distance = np.linalg.norm(np.float32(kps1[m.queryIdx].pt) - np.float32(kps2[m.trainIdx].pt))
    disp_img[pos[1], pos[0]] = distance

cv2.imshow('disp_pt', disp_img)

cv2.waitKey(0)