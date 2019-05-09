import numpy as np
import RPi.GPIO as io


class ServoHead:

    _motor1_pin = 29
    _motor2_pin = 31

    _horizontal_angle_motor2 = 22
    _top_angle_motor2 = 15

    _forward_angle_motor1 = 14
    _forward_angle_motor2 = _horizontal_angle_motor2

    _left_angle_motor1 = 23
    _left_angle_motor2 = _horizontal_angle_motor2

    _right_angle_motor1 = 4
    _right_angle_motor2 = _horizontal_angle_motor2

    _motor1_min_angle = 3
    _motor1_max_angle = 24

    _motor2_min_angle = 5
    _motor2_max_angle = 23

    _current_angle_motor1 = None
    _current_angle_motor2 = None

    def __init__(self):
        io.setmode(io.BOARD)
        io.setup(self._motor1_pin, io.OUT)
        io.setup(self._motor2_pin, io.OUT)
        self.motor1 = io.PWM(self._motor1_pin, 100)
        self.motor2 = io.PWM(self._motor2_pin, 100)
        self.motor1.start(self._forward_angle_motor1)
        self.motor2.start(self._forward_angle_motor2)
        self._current_angle_motor1 = self._forward_angle_motor1
        self._current_angle_motor2 = self._forward_angle_motor2

    def watch_forward(self):
        self.watch_angle(self._forward_angle_motor1, self._forward_angle_motor2)

    def watch_left(self):
        self.watch_angle(self._left_angle_motor1, self._left_angle_motor2)

    def watch_right(self):
        self.watch_angle(self._right_angle_motor1, self._right_angle_motor2)

    def watch_top(self):
        self.watch_angle(None, self._top_angle_motor2)

    def watch_horizontal(self):
        self.watch_angle(None, self._horizontal_angle_motor2)

    def watch_angle(self, angle1, angle2):
        if angle1 is not None:
            angle1 = np.clip(angle1, self._motor1_min_angle, self._motor1_max_angle)
            self.motor1.ChangeDutyCycle(angle1)
            self._current_angle_motor1 = angle1
        if angle2 is not None:
            angle2 = np.clip(angle2, self._motor2_min_angle, self._motor2_max_angle)
            self.motor2.ChangeDutyCycle(angle2)
            self._current_angle_motor2 = angle2

    def turn_up(self, diff=0.5):
        self.watch_angle(None, self._current_angle_motor2 - diff)

    def turn_down(self, diff=0.5):
        self.watch_angle(None, self._current_angle_motor2 + diff)

    def turn_left(self, diff=0.5):
        self.watch_angle(self._current_angle_motor1 + diff, None)

    def turn_right(self, diff=0.5):
        self.watch_angle(self._current_angle_motor1 - diff, None)

    def get_angle(self):
        return self._current_angle_motor1, self._current_angle_motor2

    def cleanup(self):
        self.motor1.stop()
        self.motor2.stop()
        del self.motor1
        del self.motor2


if __name__ == '__main__':
    sh = ServoHead()
    while True:
        k = input('op：')
        if k == 'a':
            sh.turn_left()
        elif k == 'd':
            sh.turn_right()
        elif k == 'w':
            sh.turn_up()
        elif k == 's':
            sh.turn_down()

        elif k == 'y':
            sh.watch_top()
        elif k == 'h':
            sh.watch_horizontal()
        elif k == 'g':
            sh.watch_left()
        elif k == 'j':
            sh.watch_right()
        elif k == 't':
            sh.watch_forward()

        elif k == 'q':
            break
