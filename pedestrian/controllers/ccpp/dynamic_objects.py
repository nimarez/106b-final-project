import heapq
from pathlib import Path
import numpy as np
from collections import deque
class DynamicObject(object):
    def __init__(self):
        # How far away (squared) from this object the robot must be
        self.required_distancesq = 1

    def get_traj(self, extended_occupancy_map, dt=0.5):
        # Return tuple of array of times, array of points
        pass


class Custom(DynamicObject):
    def __init__(self, trajectory, dist):
        self.trajectory = trajectory
        self.required_distancesq = dist

    def get_traj(self, extended_occupancy_map):
        return self.trajectory


class Projectile(DynamicObject):
    def __init__(self, start, velocity):
        # A projectile just moves in a straight line with constant velocity, not necessarily traveling along squares
        self.start = start
        self.velocity = velocity

    def get_traj(self, extended_occupancy_map, dt=0.5):
        traj_times, traj_points, t = [], [], 0
        curr_x, curr_y = self.start
        vel_x, vel_y = self.velocity
        while curr_x >=0 and curr_y >= 0 and curr_x < len(extended_occupancy_map[0]) \
                                        and curr_y < len(extended_occupancy_map) and not extended_occupancy_map[int(curr_y)][int(curr_x)]:
            traj_times.append(t)
            traj_points.append((curr_x, curr_y))
            t += dt
            curr_x += vel_x * dt
            curr_y += vel_y * dt
        return traj_times, traj_points



class Human(DynamicObject):
    def __init__(self, start, goal, speed):
        # start is the start square in small coordinates
        # goal is end goal in small coordinates
        self.start = start
        self.goal = goal
        self.speed = speed # squares per second I guess?
        self.directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
        self.required_distancesq = 1


    def get_traj(self, extended_occupancy_map, dt=0.5):
        # Assumes human walks in a uniform velocity along the shortest path from start to goal.
        # Assumes only static obstacles exist.
        # dt is time resolution of trajectory
        # returned trajectory is in form of dict time to square
        start_x, start_y = self.start
        goal_x, goal_y = self.goal
        if (extended_occupancy_map[start_y][start_x] or extended_occupancy_map[goal_y][goal_x]):
            raise Exception("Can't start or end in occupied square")

        # BFS
        found_goal = False
        queue = deque([self.start])
        back = {}
        visited = set()
        while queue:
            curr = queue.popleft()
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
            path = [self.start, self.goal]
            ptr = self.goal
            while back[ptr] != self.start:
                ptr = back[ptr]
                path.insert(1, ptr)
            traj_times, traj_points = [], []
            for t in list(np.arange(0, len(path) / self.speed, dt)):
                traj_times.append(round(t,3))
                traj_points.append(path[int(t * self.speed)])
            return traj_times, traj_points
        else:
            print("Goal not found")
            return None

    

            

                




                




        


