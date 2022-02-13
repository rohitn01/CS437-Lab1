from queue import PriorityQueue
import threading
import signal
import time
import numpy as np
from lab1_part2 import PiCar

NP_ROWS = 50
NP_COLS = 50

MOVE_THRESHOLD = 7


class Vertex:
    def __init__(self, parent_node=None, current_pos=None):
        self.parent_node = parent_node
        self.current_pos = current_pos

        self.g = 0
        self.h = 0
        self.f = 0
    
    def __lt__(self, other):
        if other is None:
            return False
        return self.f < other.f

def shortest_path(end):
    path = []
    # traverse up the path
    itr = end
    while itr is not None:
        path.append(itr.current_pos)
        itr = itr.parent_node

    # path is in backwards order
    return path[::-1]

def a_star(arr, start:tuple, end:tuple):
    arr_rows, arr_cols = arr.shape

    q = PriorityQueue()
    visited = set() 

    start = Vertex(None, start)
    end = Vertex(None, end)
    
    q.put(start)

    while not q.empty():
        curr_node = q.get()

        visited.add(curr_node.current_pos)

        if curr_node.current_pos == end.current_pos:
            return shortest_path(curr_node)

        for dx, dy in [(-1, 0),(0, -1),(1, 0),(0, 1)]:
            curr_x, curr_y = curr_node.current_pos
            newX, newY = curr_x + dx, curr_y + dy

            # check if it is out of bounds
            if newX >= arr_cols or newX < 0 or newY >= arr_rows or newY < 0:
                continue
            # check for obstacle
            if arr[newY, newX] == 1:
                continue
            # check if point is already visited
            if (newX, newY) in visited:
                continue
            else:
                newNode = Vertex(curr_node, (newX, newY))

                newNode.g = curr_node.g + 1
                newNode.h = heuristic(newNode.current_pos, end.current_pos)
                newNode.f = newNode.g + newNode.h

                q.put(newNode)

def heuristic(curr, end):
    x, y = curr
    g, y1 = end
    return abs(x-g)+abs(y-y1)

def pathFinder(path, target, car):
    prev = None
    dy_dx = (0, 0)
    prev_angle = car.get_orientation()
    steps_move = 0
    if path is None:
        return None
    for curr in path:
        print(curr)
        if prev == None:
            prev = curr
            continue
        move = (curr[0]-prev[0], curr[1]-prev[1])
        dy_dx = (dy_dx[0]+move[0], dy_dx[1]+move[1])
        curr_angle = 0
        if move[0] == -1:       # left
            curr_angle = 180
        elif move[0] == 1:      # right
            curr_angle = 0
        elif move[1] == 1:     # down
            curr_angle = 270
        elif move[1] == -1:      # up
            curr_angle = 90

        if curr_angle != prev_angle:
            car.move_distance(steps_move)
            car.change_angle_90(curr_angle)
            steps_move = 1
            break
        else:
            steps_move += 1
        prev_angle = curr_angle
        prev = curr
    car.move_distance(steps_move)
    return dy_dx

def moveToDestination(x, y, car):
    car_origin = (int(car.get_x()), int(car.get_y()))
    dy_dx = (0,0)
    while (x,y) != car_origin:
        car.map_car()
        grid = car.get_env_map()
        print("Received map")
        if grid[y][x] == 1:
            print("UNABLE TO GET TO LOCATION: PATH OBSTRUCTED")
            car.change_angle_90(180)
            car.change_angle_90(180)
        path = a_star(grid, car_origin, (x, y))
        print(path)
        if path is None:
            print("unable to reach path")
        if (x,y) not in path:
            print("unable to reach path")
            break
        dy_dx = pathFinder(path[:MOVE_THRESHOLD], (x, y), car)
        car_origin = (int(car.get_x()), int(car.get_y()))

if __name__ == "__main__":
    grid = np.load("env_map_test.npy")

    car = PiCar()
    moveToDestination(24, 24, car)
    print(car.get_x(), car.get_y())    

    
