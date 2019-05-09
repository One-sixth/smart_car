import RPi.GPIO as io
import time


class UltrasoundRanger:
    tri_pin = 33
    echo_pin = 32

    # ms单位
    max_delay = 60
    inter_delay = 5

    def __init__(self):
        io.setmode(io.BOARD)
        io.setup(self.tri_pin, io.OUT, initial=io.LOW)
        io.setup(self.echo_pin, io.IN, pull_up_down=io.PUD_DOWN)

    def detect(self):
        # 给予至少10us的高电平，发送信号
        io.output(self.tri_pin, io.HIGH)
        time.sleep(0.00001)
        io.output(self.tri_pin, io.LOW)
        # 等待回声，echo 高电平的持续时间就是声音发出时间与接收时间的间隔
        b = io.wait_for_edge(self.echo_pin, io.RISING, timeout=self.max_delay)
        if b is None:
            # 没有回声
            return None
        start_time = time.perf_counter()
        # 等待高电平结束
        b = io.wait_for_edge(self.echo_pin, io.FALLING, timeout=self.max_delay*10)
        if b is None:
            # 这不应该发生，检查你的连线
            # 太近也会发生这种情况
            print('UltrasoundRanger: wrong! too close or check you ultrasound link!')
            return 0
        end_time = time.perf_counter()
        # 得到的是cm单位
        le = (end_time - start_time) * 343 * 100 / 2
        return le

    def cleanup(self):
        io.remove_event_detect(self.echo_pin)


if __name__ == '__main__':
    ur = UltrasoundRanger()
    while True:
        le = ur.detect()
        if le is not None:
            print('detect len %.2f' % le)
        else:
            print('detect len inf')
        time.sleep(0.5)
