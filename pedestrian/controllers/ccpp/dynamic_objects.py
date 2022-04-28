import heapq
from pathlib import Path
import numpy as np
class DynamicObject(object):
    def __init__(self):
        pass

    def get_traj(self):
        pass

class Human(DynamicObject):
    def __init__(self, start, goal, velocity):
        # start is the start square in small coordinates
        # goal is end goal in small coordinates
        self.start = start
        self.goal = goal
        self.velocity = velocity # squares per second I guess?
        self.directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
    
    def get_traj(self, extended_occupancy_map, dt=0.5):
        # Assumes human walks in a uniform velocity along the shortest path from start to goal.
        # Assumes only static obstacles exist.
        # dt is time resolution of trajectory
        # returned trajectory is in form of dict time to square
        start_x, start_y = self.start
        goal_x, goal_y = self.goal
        if (extended_occupancy_map[start_y][start_x] or extended_occupancy_map[goal_y][goal_x]):
            raise Exception("Can't start in occupied square")

        # BFS
        found_goal = False
        queue = [self.start]
        back = {}
        visited = set()
        while queue:
            curr = queue.pop(0)
            if (curr == self.goal):
                found_goal = True
                break
            visited.add(curr)
            for (dx, dy) in self.directions:
                curr_x, curr_y = curr
                next_x, next_y = curr_x + dx, curr_y + dy
                if next_x < 0 or next_y < 0 or next_x >= len(extended_occupancy_map[0]) or next_y >= len(extended_occupancy_map):
                    continue
                if not extended_occupancy_map[next_y][next_x] and (next_x, next_y) not in visited:
                    queue.append((next_x, next_y))
                    back[(next_x, next_y)] = curr
                    
        # retrace path
        if found_goal:
            path = [self.goal]
            ptr = self.goal
            while back[ptr] != self.start:
                ptr = back[ptr]
                path.append(ptr)
            path.append(self.start)
            path.reverse()
            traj = {}
            for t in list(np.arange(0, len(path) / self.velocity, dt)):
                traj[round(t,3)] = path[int(t * self.velocity)]
            return traj
        else:
            print("Goal not found")
            return None

            

                




                




        


