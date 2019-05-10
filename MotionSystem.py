'''
这里是动力系统，有控制两种办法。
一种是超时马力，一种是基于slam检查的单位距离
基于slam检查系统工作原理，slam观察到移动一单位距离时的timeout值
仅建议在地图比较稳定的时候使用
'''

# import numpy as np
import RPi.GPIO as io

class MotionSystem:
    '''
    timeout 单位为毫秒
    '''

    motor0 = [11, 7]
    motor1 = [12, 13]
    motor2 = [16, 15]
    motor3 = [18, 22]
    motors = [motor0, motor1, motor2, motor3]

    motor_pwm = []

    status_forward = 1
    status_stop = 0
    status_back = -1

    def __init__(self):
        io.setmode(io.BOARD)
        for motor in self.motors:
            io.setup(motor[0], io.OUT)
            io.setup(motor[1], io.OUT)
        self.cur_status = 0
        self.stop()

        self.motor_pwm = [
            [io.PWM(self.motor0[0], 1), io.PWM(self.motor0[1], 100)],
            [io.PWM(self.motor1[0], 1), io.PWM(self.motor1[1], 100)],
            [io.PWM(self.motor2[0], 1), io.PWM(self.motor2[1], 100)],
            [io.PWM(self.motor3[0], 1), io.PWM(self.motor3[1], 100)]
        ]

        for m in self.motor_pwm:
            for p in m:
                p.start(0)

    def to_status(self, wait_to_status):
        '''用于减少状态切换次数'''
        if self.cur_status == wait_to_status:
            return False
        self.cur_status = wait_to_status
        return True

    def motor_controller(self, motor_id, status, speed=1.):
        # if status == self.status_forward:
        #     io.output(self.motors[motor_id][0], 0)
        #     io.output(self.motors[motor_id][1], 1)
        # elif status == self.status_stop:
        #     io.output(self.motors[motor_id][0], 0)
        #     io.output(self.motors[motor_id][1], 0)
        # elif status == self.status_back:
        #     io.output(self.motors[motor_id][0], 1)
        #     io.output(self.motors[motor_id][1], 0)

        if status == self.status_forward:
            self.motor_pwm[motor_id][0].ChangeDutyCycle(0)
            self.motor_pwm[motor_id][1].ChangeDutyCycle(speed*100)
        elif status == self.status_stop:
            self.motor_pwm[motor_id][0].ChangeDutyCycle(0)
            self.motor_pwm[motor_id][1].ChangeDutyCycle(0)
        elif status == self.status_back:
            self.motor_pwm[motor_id][0].ChangeDutyCycle(speed*100)
            self.motor_pwm[motor_id][1].ChangeDutyCycle(0)

    def stop(self):
        if self.to_status(0):
            self.motor_controller(0, self.status_stop)
            self.motor_controller(1, self.status_stop)
            self.motor_controller(2, self.status_stop)
            self.motor_controller(3, self.status_stop)

    def forward(self):
        if self.to_status(1):
            self.motor_controller(0, self.status_forward)
            self.motor_controller(1, self.status_forward)
            self.motor_controller(2, self.status_forward)
            self.motor_controller(3, self.status_forward)

    def back(self):
        if self.to_status(2):
            self.motor_controller(0, self.status_back)
            self.motor_controller(1, self.status_back)
            self.motor_controller(2, self.status_back)
            self.motor_controller(3, self.status_back)

    def left(self):
        if self.to_status(3):
            self.motor_controller(0, self.status_forward, 0.1)
            self.motor_controller(1, self.status_forward)
            self.motor_controller(2, self.status_forward, 0.1)
            self.motor_controller(3, self.status_forward)

    def right(self):
        if self.to_status(4):
            self.motor_controller(0, self.status_forward)
            self.motor_controller(1, self.status_forward, 0.1)
            self.motor_controller(2, self.status_forward)
            self.motor_controller(3, self.status_forward, 0.1)

    # def left(self):
    #     if self.to_status(3):
    #         self.motor_controller(0, self.status_back)
    #         self.motor_controller(1, self.status_forward)
    #         self.motor_controller(2, self.status_back)
    #         self.motor_controller(3, self.status_forward)
    #
    # def right(self):
    #     if self.to_status(4):
    #         self.motor_controller(0, self.status_forward)
    #         self.motor_controller(1, self.status_back)
    #         self.motor_controller(2, self.status_forward)
    #         self.motor_controller(3, self.status_back)


if __name__ == '__main__':
    ms = MotionSystem()
    while True:
        k = input('op：')
        if k == 'a':
            ms.left()
        elif k == 'd':
            ms.right()
        elif k == 'w':
            ms.forward()
        elif k == 's':
            ms.back()
        elif k == ' ':
            ms.stop()
        elif k == 'q':
            break