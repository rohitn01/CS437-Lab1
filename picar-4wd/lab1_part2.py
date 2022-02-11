import picar_4wd as fc
from picar_4wd.speed import Speed
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.pin import Pin
import time
import signal
import sys
import numpy as np

DIST_TO_STOP = 15
SERVO_ANGLE_RANGE = 80
SERVO_ANGLE_STEP = 10

MAX_GRID_SIZE = 50
class PiCar():
    def __init__(self):
        self.orientation = -90
        self.position = (int(MAX_GRID_SIZE/2)-1,0)
        self.max_angle_bound = SERVO_ANGLE_RANGE
        self.angles = np.array([i for i in range(-int(self.max_angle_bound), int(self.max_angle_bound) + SERVO_ANGLE_STEP, SERVO_ANGLE_STEP)])
        self.scans = []
        
        self.current_angle = 0
        self.scan_list = []

        self.env_grid = np.zeros((MAX_GRID_SIZE,MAX_GRID_SIZE))
        self.env_grid[0][int(MAX_GRID_SIZE/2)-1] = -1

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
                
    def scan_step(self):
        angle_steps = self.angles.copy()
        scan_reversed = False
        if self.current_angle == self.max_angle_bound:
            angle_steps = np.flip(angle_steps)
            scan_reversed = True

        scan_list = []
        for angle in angle_steps:
            time.sleep(0.2)
            dist = fc.get_distance_at(angle)
            self.current_angle = angle
            scan_list.append(dist)
        

        if scan_reversed:
            scan_list.reverse()
        # print(self.scan_list)
        return np.array(scan_list)

    def map_car(self):
        cosines = np.cos(np.radians(self.angles+self.orientation))
        sines = np.sin(np.radians(self.angles+self.orientation))
        num_scans = 0
        while num_scans < 1:
            scan = self.scan_step()
            prevIdx = -1
            prev = 0
            prev_row = -1
            prev_col = -1
            scan_sines = scan * sines
            scan_cosines = scan * cosines
            print(scan)
            for i in range(len(scan)):
                if scan[i] < 0:
                    prev = 0
                    continue
                row = int(self.position[1] - scan_sines[i]/9)
                col = int(self.position[0] + scan_cosines[i]/9)
                print((row, col, scan_cosines[i]))
                if row < 0 or col < 0 or col >= 50 or row >= 50:
                    print("Out of Bounds!")
                    prev = 0
                    self.env_grid[row][col] = 0
                    continue
                
                print(i+1)
                self.env_grid[row][col] = 1
                # create line between previous and current point
                if prev > 0:
                    print("Last: " + str(prevIdx) + " " + str(prev))
                    if col == prev_col:
                        for j in range(prev_row, row):
                            self.env_grid[j][col] = 1
                    else:
                        slope = (row - prev_row) / (col - prev_col)
                        for j in range(min(prev_col, col), max(prev_col, col)):
                            y = int(prev_row + slope * (j - prev_col))
                            self.env_grid[y][j] = 1

                prevIdx = i+1
                prev = 1
                prev_row = row
                prev_col = col
            num_scans += 1

    def print_env_map(self):
        print(self.env_grid.shape)
        np.savetxt("env_map_test.npy", self.env_grid, delimiter=" ", fmt='%i')
car = PiCar()

car.map_car()
car.print_env_map()

