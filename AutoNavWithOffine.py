import math
from UltrasonicRadar import UltrasonicRadar
from MotionSystem import MotionSystem
import time


class AutoNavWithOffline:
    def __init__(self, nav_type='follow_left'):
        self.nav_type = nav_type

    def nav(self, motion_system: MotionSystem, us_radar: UltrasonicRadar):
        if us_radar.check_forward():
            motion_system.forward()
            time.sleep(0.5)
            motion_system.stop()
        elif us_radar.check_right():
            motion_system.right()
            time.sleep(0.5)
            motion_system.stop()
