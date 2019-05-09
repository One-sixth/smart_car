import cv2
import numpy as np
import imageio

scene_img = imageio.imread(r"D:\1myworkspace\ミク.jpg")
scene_img = cv2.resize(scene_img, (1280, 720))
scene_img = cv2.cvtColor(scene_img, cv2.COLOR_RGB2BGR)

obj_img = imageio.imread(r"D:\1myworkspace\ミク2.jpg")
obj_img = cv2.resize(obj_img, (480, 240))
obj_img = cv2.cvtColor(obj_img, cv2.COLOR_RGB2BGR)

orb = cv2.ORB_create(1000, 4, 4)

scene_kp, scene_des = orb.detectAndCompute(scene_img, None)
obj_kp, obj_des = orb.detectAndCompute(obj_img, None)

obj_des = np.float32(obj_des)
scene_des = np.float32(scene_des)

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(obj_des, scene_des, k=2)

good_matches = []
for m, n in matches:
    if m.distance < 0.7*n.distance:
        good_matches.append(m)

img_matches = cv2.drawMatches(obj_img, obj_kp, scene_img, scene_kp, good_matches, None) #, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

# src_pts = np.float32([obj_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
# dst_pts = np.float32([scene_kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
# M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
# matchesMask = mask.ravel().tolist()
# h, w, d = obj_img.shape
# pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
# dst = cv2.perspectiveTransform(pts, M)
# img_matches = cv2.polylines(img_matches, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
#
#
cv2.imshow("Good Matches & Object detection", img_matches)

cv2.waitKey(0)
