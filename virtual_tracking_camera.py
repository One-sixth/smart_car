import cv2
import numpy as np
import imageio


class virtual_tracking_camera:
    def __init__(self):
        self.img = imageio.imread(r"D:\1myworkspace\ミク.jpg")
        self.img = cv2.resize(self.img, (1280, 720))
        self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        cv2.imshow('cam', self.img)
        self.is_quit = False
        self.trmat = np.eye(3)[:2]
        self.orb = cv2.ORB_create(2000, 6, 5)

    def up(self):
        self.trmat[1][2] -= 1

    def down(self):
        self.trmat[1][2] += 1

    def left(self):
        self.trmat[0][2] -= 1

    def right(self):
        self.trmat[0][2] += 1

    def zoomIn(self):
        self.trmat[0][0] += 0.1
        self.trmat[1][1] += 0.1

        self.trmat[0][2] -= self.img.shape[1] / 2 * 0.1
        self.trmat[1][2] -= self.img.shape[0] / 2 * 0.1

    def zoomOut(self):
        self.trmat[0][0] -= 0.1
        self.trmat[1][1] -= 0.1

        self.trmat[0][2] += self.img.shape[1] / 2 * 0.1
        self.trmat[1][2] += self.img.shape[0] / 2 * 0.1

    def draw(self):
        nim = cv2.warpAffine(self.img, self.trmat, self.img.shape[:2][::-1], borderMode=cv2.BORDER_CONSTANT, borderValue=0)

        kp, des = self.orb.detectAndCompute(nim, None)
        img2 = nim.copy()
        for marker in kp:
            img2 = cv2.drawMarker(img2, tuple(int(i) for i in marker.pt), color=(0, 255, 0))
        # img2 = cv2.drawKeypoints(img, kps, None, color=(0, 255, 0), flags=0)
        cv2.imshow('viewer', img2)

        cv2.imshow('cam', nim)

    def run(self):
        while not self.is_quit:
            self.draw()
            key = cv2.waitKey(1000//30)
            if key == ord('w'):
                self.up()
            elif key == ord('s'):
                self.down()
            elif key == ord('a'):
                self.left()
            elif key == ord('d'):
                self.right()
            elif key == ord('q'):
                self.zoomOut()
            elif key == ord('e'):
                self.zoomIn()


if __name__ == '__main__':
    vtcam = virtual_tracking_camera()
    vtcam.run()
