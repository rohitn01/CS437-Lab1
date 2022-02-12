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
MOTOR_POWER = 4
TURN_TIME = 580
MAX_GRID_SIZE = 50
class PiCar():
    def __init__(self):
        self.orientation_angles = set([0, 90, 180, 270])
        self.orientation = 0
        self.position = (0,int(MAX_GRID_SIZE/2)-1)
        self.max_angle_bound = SERVO_ANGLE_RANGE
        self.angles = np.array([i for i in range(-int(self.max_angle_bound), int(self.max_angle_bound) + SERVO_ANGLE_STEP, SERVO_ANGLE_STEP)])
        self.scans = []
        
        self.current_angle = 0
        self.scan_list = []

        self.env_map = np.zeros((MAX_GRID_SIZE,MAX_GRID_SIZE))

        signal.signal(signal.SIGINT, self.handle_signal)
        
    def handle_signal(self, signum, frame):
        fc.stop()
        sys.exit(0)

    def change_angle_90(self, new_angle):
        
        if new_angle not in self.orientation_angles:
            return
        
        print(new_angle, self.orientation)
        
        angle_change = new_angle - self.orientation
        print(angle_change)
        if angle_change == -90 or angle_change == 270:
            self.turn_right_90()
        elif angle_change == 90 or angle_change == -270:
            self.turn_left_90()
        elif angle_change == 180:
            self.turn_left_90()
            time.sleep(1)
            self.turn_left_90()
        elif angle_change == -180:
            self.turn_right_90()
            time.sleep(1)
            self.turn_right_90()
        self.orientation = new_angle
        time.sleep(1)

    def turn_left_90(self):
        fc.turn_right(MOTOR_POWER)
        time.sleep(1.1)
        fc.stop()

    def turn_right_90(self):
        fc.turn_left(MOTOR_POWER)
        time.sleep(1.1)
        fc.stop()

    def move_distance(self, grid_dist):
        dist_in_cm = 9 * grid_dist

        dist_travelled = 0
        time.sleep(0.005)
        fc.backward(4)
        while dist_travelled < dist_in_cm:
            time.sleep(0.005)
            dist_travelled += .15
        fc.stop()

        self.position = (self.position[0] + (dist_in_cm * np.cos(self.orientation))/9, self.position[1] + (dist_in_cm * np.sin(self.orientation)/9))
    
    def get_x(self):
        return self.position[0]
    def get_y(self):
        return self.position[1]

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
        print("MAPPING")
        self.env_map = np.zeros((MAX_GRID_SIZE, MAX_GRID_SIZE))
        self.env_map[int(self.get_y())][int(self.get_x())] = 8
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
            for i in range(len(scan)):
                if scan[i] < 0:
                    prev = 0
                    continue
                row = int(self.position[1] - scan_sines[i]/9)
                col = int(self.position[0] + scan_cosines[i]/9)
                if self.orientation == 0:
                    col -= 1
                elif self.orientation == 90:
                    row += 1
                elif self.orientation == 180:
                    col += 1
                elif self.orientation == 270:
                    row -= 1
                if row < 0 or col < 0 or col >= 50 or row >= 50:
                    prev = 0
                    continue
                
                self.env_map[row][col] = 1
                # create line between previous and current point
                if prev > 0:
                    if col == prev_col:
                        for j in range(prev_row, row):
                            self.env_map[j][col] = 1
                    else:
                        slope = (row - prev_row) / (col - prev_col)
                        for j in range(min(prev_col, col), max(prev_col, col)):
                            y = int(prev_row + slope * (j - prev_col))
                            self.env_map[y][j] = 1

                prevIdx = i+1
                prev = 1
                prev_row = row
                prev_col = col
            num_scans += 1
        np.savetxt("env_map_test_txt.npy", self.env_map, delimiter=" ", fmt='%i')

    def print_env_map(self):
        print(self.env_map.shape)
        np.save("env_map_test.npy", self.env_map)
        np.savetxt("env_map_test_txt.npy", self.env_map, delimiter=" ", fmt='%i')

    def get_env_map(self):
        return self.env_map



