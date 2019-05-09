from car_op import CarOp
import numpy as np


class AutoWatchHuman:
    def __init__(self, car_op: CarOp):
        self.car_op = car_op
        self.hw_center = np.array([360//2, 640//2])

    def auto_watch_run(self):
        pass

    def detect(self, img, boxes: np.array):
        if len(boxes) == 0:
            self.car_op.ctrl_servohead('forward')
            return

        boxes = np.array(boxes, np.float)
        # 只看最大的框
        area = np.prod(np.abs(boxes[:, 2:] - boxes[:, 2]))
        big_box = boxes[np.argmax(area)]
        cent = big_box[:2] + (big_box[2:] - big_box[:2]) * 2

        diff_hw = cent - self.hw_center
        dy, dx = diff_hw
        if abs(dy) > abs(dx):
            if dy > 0:
                self.car_op.ctrl_servohead('turn_down')
            else:
                self.car_op.ctrl_servohead('turn_down')
        else:
            if dx > 0:
                self.car_op.ctrl_servohead('turn_left')
            else:
                self.car_op.ctrl_servohead('turn_right')
