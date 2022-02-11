import picar_4wd as fc
from picar_4wd.speed import Speed
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pin import Pin
import time
import signal
import sys
import numpy as np

DIST_TO_STOP = 15
class PiCar():
    def __init__(self):
        self.env_map = np.zeros((100,100))
        self.displacement = (50,50)
        self.orientation = 90
        
        signal.signal(signal.SIGINT, self.handle_signal)
        
    def handle_signal(self, signum, frame):
        fc.stop()
        sys.exit(0)

    def change_angle(self):
        rand = np.random.randint(0,2)
        
        fc.backward(4)
        
        if rand == 1:
            fc.turn_left(4)
        else:
            fc.turn_right(4)
            
        time.sleep(np.random.rand())
        
        fc.stop()
        

    def detect_obj_and_stop(self):
        us = Ultrasonic(Pin("D8"), Pin("D9"))
        fc.backward(4)
        while True:
            dis_val = us.get_distance()
            time.sleep(0.1)
            print(dis_val)
            if -1 <= dis_val <= DIST_TO_STOP:
                fc.stop()
                fc.forward(4)
                fc.stop()
                self.change_angle()
                fc.backward(4)

car = PiCar()

car.detect_obj_and_stop()
