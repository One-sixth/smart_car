from car_op import CarOp
import numpy as np


class AutoWatchHuman:
    def __init__(self, car_op: CarOp):
        self.car_op = car_op
        # self.hw_center = np.array([360//2, 640//2])

    def auto_watch_run(self):
        pass

    def detect(self, img, boxes):
        if len(boxes) == 0:
            self.car_op.ctrl_servohead('forward')
            return

        a, b = boxes[:, 1].copy(), boxes[:, 0].copy()
        boxes[:, 0], boxes[:, 1] = a, b
        a, b = boxes[:, 3].copy(), boxes[:, 2].copy()
        boxes[:, 2], boxes[:, 3] = a, b

        hw_center = (img.shape[0] // 2, img.shape[1] // 2)

        boxes = np.array(boxes, np.float)
        # 只看最大的框
        area = np.prod(np.abs(boxes[:, 2:] - boxes[:, :2]))
        big_box = boxes[np.argmax(area)]
        center = big_box[:2] + (big_box[2:] - big_box[:2]) / 2

        diff_hw = center - hw_center
        # diff_hw = diff_hw / img.shape[:2]
        dy, dx = diff_hw

        print(diff_hw)
        # if abs(dy) > abs(dx):
        if dy > 0:
            self.car_op.ctrl_servohead('turn_down')
            print('turn_down')
        else:
            self.car_op.ctrl_servohead('turn_up')
            print('turn_up')
        # else:
        if dx > 0:
            self.car_op.ctrl_servohead('turn_right')
            print('turn_right')
        else:
            self.car_op.ctrl_servohead('turn_left')
            print('turn_left')
