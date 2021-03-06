from tkinter import N
import matplotlib.pyplot as plt
import numpy as np
import random
from dynamic_objects import DynamicObject, Human, Projectile, Custom


class Planner(object):
    """
    Plans a complete coverage path through an obstacle field in the presence of dynamic obstacles
    See functions for more documentation

    Extended occupancy map: 2d matrix of integers, 0=unoccupied, 1=obstacle, 0.5=already completed
    """
    def __init__(self, dim, straight_vel, turn_time):
        # Straight_vel = grid squares per second of velocity
        # Turn_time = time to turn 90 degrees
        # Dim is a tuple for size of the world
        self.dim = dim
        # Max iters
        self.max_iters = 100
        # Up left down right
        self.coords_to_check = [(0, -1), (-1, 0), (0, 1), (1, 0)]
        # 1/Velocity -- how many squares per second does the robot move
        self.dt = 1/straight_vel
        self.turn_time = turn_time


    def visualize_plan(self, extended_occupancy_map, dynamic_objects, plan, t_step=4):
        modified_occ = 1/np.array(extended_occupancy_map)
        modified_occ[modified_occ == np.inf] = 0
        plt.matshow(modified_occ)
        trajectories = [obj.get_traj(extended_occupancy_map) for obj in dynamic_objects]
        is_plan = True
        for (times, points) in [plan] + trajectories:
            x = [p[0] for p in points]
            y = [p[1] for p in points]
            if is_plan:
                color = "b-"
                textcolor = "aquamarine"
            else:
                color = "r-"
                textcolor = "lightcoral"
            plt.plot(x, y, color) 
            for i, (xitem, yitem, t) in enumerate(zip(x, y, times)):
                if t_step > 0 and i % t_step == 0:
                    plt.text(xitem, yitem, str(t), color=textcolor)
            is_plan = False
        plt.gca().set_xticks([x - 0.5 for x in plt.gca().get_xticks()][1:], minor='true')
        plt.gca().set_yticks([y - 0.5 for y in plt.gca().get_yticks()][1:], minor='true')
        #plt.grid(which='minor')
        plt.show()

    def _generate_spanning_tree(self, large_map, start_pos, previous_tree=None):
        """
        Generates a (not minimal) spanning tree, returns a dictionary of (pos) --> [boolean for up down left right if it connects]
        large_map: Returned from _largify_cells, binary matrix for large cells for spanning tree algorithm
        start_pos: tuple of (x, y) starting position in SMALL coordinates
        NOTE: This is not deterministic, we can generate random trees each time
        NOTE: Edges are only one way, from parent to child
        """
        # Previous tree should just be a line -- assert this before
        if previous_tree is not None:
            start_pos_big = (start_pos[0] // 2, start_pos[1] // 2)
            curr, curr_val = (start_pos_big, previous_tree[start_pos_big])
            while any(curr_val):
                index = curr_val.index(True)
                curr = (curr[0] + self.coords_to_check[index][0], curr[1] + self.coords_to_check[index][1])
                curr_val = previous_tree[curr]
                if sum(curr_val) > 1:
                    raise Exception("Previous tree isn't a line:(")
        to_visit = [(None, None, start_pos)]
        edges = {}
        visited = set()
        while to_visit:
            parent, parent_index, current = to_visit.pop()
            #print("Visiting", current)
            if current in visited:
                continue
            if current not in edges:
                edges[current] = [False, False, False, False]
            if parent is not None:
                edges[parent][parent_index] = True
            visited.add(current)
            c = list(zip(range(4), self.coords_to_check[:]))
            random.shuffle(c)
            for i, (xdiff, ydiff) in c:
                newx, newy = current[0] + xdiff, current[1] + ydiff
                if newx < 0 or newy < 0 \
                   or newx >= len(large_map[0]) or newy >= len(large_map) \
                   or large_map[newy][newx] != 0 or (newx, newy) in visited:
                    continue
                to_visit.append((current, i, (newx, newy)))
            # If we have a previous tree (line), make sure the edges are appended last so they get visited first
            #   and thus added to the tree
            if previous_tree is not None and current in previous_tree and True in previous_tree[current]:
                index = previous_tree[current].index(True)
                newcoord = current[0] + self.coords_to_check[index][0], current[1] + self.coords_to_check[index][1]
                to_visit.append((current, index, newcoord))
        return edges

    def _largify_cells(self, extended_occupancy_map):
        """
        Takes in an extended occupancy map and returns a binary occupancy map in large coordinates (2x2 squares)
        Cuts off the edges if odd-sized, and marks partial occupancy of obstacles as full occupancy
        Partial occupancy of previously visited cells is marked as unvisited (due to handling of previous paths)
        """
        # Nonzero means either already visited or obstacle, so we just mark as an obstacle essentially
        result = []
        for y in range(len(extended_occupancy_map) // 2):
            row = []
            for x in range(len(extended_occupancy_map[0]) // 2):
                s1 = extended_occupancy_map[2*y][2*x]
                s2 = extended_occupancy_map[2*y + 1][2*x]
                s3 = extended_occupancy_map[2*y][2*x + 1]
                s4 = extended_occupancy_map[2*y + 1][2*x + 1]
                any_objects = np.any(np.array([s1, s2, s3, s4]) == np.ones(4))
                all_filled = np.all(np.array([s1, s2, s3, s4]) == 2*np.ones(4))
                row.append(int(any_objects or all_filled))
            result.append(row)
        return result

    def _prune_tree(self, tree, start_pos, current_position):
        """
        Calculates the tree prefix which must be preserved when moving from the current position
        Prunes the tree by removing all nodes in unfilled or filled squares
        This is useful because we can mark the fully visited nodes as obstacles essentially
            It costs nothing to go around them because the paths are all uniform length because space-filling
        Also removes the unvisited squares, because we want to regenerate the path if we encounter a dynamic obstacle 
        tree: Returned from generated spanning tree
        start_position: Start location
        current_position: Current location in tree
        Returns tuple of (set of completed small squares, pruned tree)
        """
        # NOTE: Assumes that walk tree is deterministic (!!)
        path_points = self._walk_tree(tree, start_pos)
        current_index = path_points.index(current_position)
        completed_points = set(path_points[:current_index + 1])
        # Returns the set of unfilled squares, completely filled squares, and partially filled squares
        partial = set()
        for (x, y) in tree:
            p0 = (2*x, 2*y) in completed_points
            p1 = (2*x + 1, 2*y) in completed_points
            p2 = (2*x, 2*y + 1) in completed_points
            p3 = (2*x + 1, 2*y + 1) in completed_points
            arr = [p0, p1, p2, p3]
            if sum(arr) > 0 and sum(arr) < 4:
                partial.add((x, y))
        pruned_tree = {}
        current = start_pos
        while current is not None:
            partial_edges = [(current[0] + offset[0], current[1] + offset[1]) in partial for offset in self.coords_to_check]
            both = [a and b for (a, b) in zip(tree[current], partial_edges)]
            pruned_tree[current] = [False, False, False, False]
            if sum(both) == 0:
                # No edges satisfy, we're done
                current = None
            elif sum(both) == 1:
                index = both.index(True)
                pruned_tree[current][index] = True
                offset = self.coords_to_check[index]
                current = (current[0] + offset[0], current[1] + offset[1])
            else:
                # This shouldn't happen
                raise Exception("Not a line potential:(")
        return completed_points, pruned_tree

    def _walk_tree(self, tree, start_pos):
        """
        Takes in a tree (in large coordinates) and start_pos (in small coordinates !!) and produces a path from walking that tree
        """
        stack = [(start_pos[0] // 2, start_pos[1] // 2)]
        previous = None
        visited = set()
        # xmod, ymod --> next xdiff, next ydiff
        # The next one counterclockwise
        next_map = {
            (0, 0): (0, 1),
            (0, 1): (1, 0),
            (1, 1): (0, -1),
            (1, 0): (-1, 0)
        }
        # Turning in a corner, need special case
        next_map_right_turn = {
            (0, 0): (-1, 0),
            (0, 1): (0, 1),
            (1, 1): (1, 0),
            (1, 0): (0, -1)
        }
        # We shift the next direction to check based on where we are in the square
        # For instance, at top right we check the up direction first
        coord_check_shift = {
            (0, 0): 1,
            (0, 1): 2, 
            (1, 1): 3,
            (1, 0): 0
        }
        path = [start_pos]
        while stack:
            current = stack.pop()
            # Don't check for visited here because we're probably already visited
            visited.add(current)
            current_position = path[-1]
            xmod, ymod = current_position[0] % 2, current_position[1] % 2
            #print("Big square", current, "previously at", current_position)
            if previous is not None:
                # NOTE: This might fail when we have a tree of size 1 because previous is never set
                # Finds the counterclockwise shortest path from the last position to the closest position in the new square
                dx, dy = next_map_right_turn[(xmod, ymod)]
                right_turn_pos = (current_position[0] + dx, current_position[1] + dy)
                if right_turn_pos[0] // 2 == current[0] and right_turn_pos[1] // 2 == current[1]:
                    # Turns stay in the same square
                    path.append(right_turn_pos)
                else:
                    while True:
                        xmod, ymod = current_position[0] % 2, current_position[1] % 2
                        dx, dy = next_map[(xmod, ymod)]
                        current_position = (current_position[0] + dx, current_position[1] + dy)
                        path.append(current_position)
                        # Stop when, if we continue in the same direction, we end up in the target square
                        new_position = (current_position[0] + dx, current_position[1] + dy)
                        if new_position[0] // 2 == current[0] and new_position[1] // 2 == current[1]:
                            path.append(new_position)
                            break
            # Get position again after moving into this square so we know which direction to go next
            current_position = path[-1]
            xmod, ymod = current_position[0] % 2, current_position[1] % 2
            shift = coord_check_shift[(xmod, ymod)]
            shifted_coords = self.coords_to_check[shift:] + self.coords_to_check[:shift]
            for unshifted_i, (xdiff, ydiff) in enumerate(shifted_coords):
                index = (unshifted_i + shift) % 4
                #print("Checking", ["up", "left", "down", "right"][index], "for", current_position)
                #print((xdiff, ydiff))
                new_pos = (current[0] + xdiff, current[1] + ydiff)
                if tree[current][index] and new_pos not in visited:
                    # We want to visit current against at the end, and repeat the same thing
                    stack.append(current)
                    stack.append(new_pos)
                    #print("Ending")
                    break
            previous = current
        return path

    def _generate_plan(self, extended_occupancy_map, start_pos, previous_tree_pruned=None, current_pos=None, current_time=None):
        """
        Generates a path (tree, list of times, list of positions) through an occupancy map
        If the previous tree and time are set, then generates a path compatible with the previous one and starts at the current time
        Extended occupancy map is same as occupancy map but with the potential for a square to be already explored
        Start pos is position in SMALL COORDINATES
        Dynamic objects is a representation of human intent, e.g. list of dict of time to coordinate
        When starting out, current_pos and previous_tree are not needed
            Can set these to regenerate part of the path
        Thing to consider: we must complete the other side of squares we already visited first, if we decide to replan
        - Solution: Replan given the current completed prefix of the plan,
            record big squares as being fully cleaned, partially cleaned, etc.
        Future work:
        - Make spanning tree thing work for obstacles which only cover 1/4 of the mega cell
        """
        large_cells = self._largify_cells(extended_occupancy_map)
        tree = self._generate_spanning_tree(large_cells, start_pos, previous_tree_pruned)
        path = self._walk_tree(tree, start_pos)
        # Assumes the robot moves with constant velocity of 1 square/self.dt
        times = []
        points = []
        if current_time is None:
            time = 0
        else:
            path = path[path.index(current_pos):]
            time = current_time
        for i in range(len(path)):
            if i > 2:
                curr = path[i]
                prev = path[i - 1]
                pprev = path[i - 2]
                diff0 = (curr[0] - prev[0], curr[1] - prev[1])
                diff1 = (prev[0] - pprev[0], prev[1] - pprev[1])
                # If we turn, add turn compensation
                if diff0[0] != diff1[0] or diff0[1] != diff1[1]:
                    time += self.turn_time
            times.append(time)
            points.append(path[i])
            time += self.dt
        return tree, times, points

    def generate_compatible_plan(self, extended_occupancy_map, dynamic_objects, start_pos, current_pos=None, current_time=None, previous_tree=None):
        """
        Calls generate_plan until one is found which doesn't intersect with the dynamic objects
        Additionally updates the occupancy map with the previously completed path
        Returns tuple of (complete path found, updated occupancy map with filled in squares from the previous run, tree, plan times, plan points)
        """
        pruned_tree = None
        complete = None
        # Duplicate occupancy
        new_occ = [x[:] for x in extended_occupancy_map]
        if current_pos is not None:
            complete, pruned_tree = self._prune_tree(previous_tree, start_pos, current_pos)
            for point in complete:
                new_occ[point[1]][point[0]] = 2
        # NOTE: Assumes that the plan is generated in intervals of self.dt
        # It should also be deterministic constant length
        dynamic_object_positions = None
        longest_path = None
        longest_length = -1
        for i in range(self.max_iters):
            #print("Trying iteration", i)
            tree, times, points = self._generate_plan(new_occ, start_pos, pruned_tree, current_pos, current_time)
            if dynamic_object_positions is None:
                # We only need to set this once
                # Calculate where the dynamic objects are at each iteration of dt
                # We assume that the waypoints are close enough that we can snap to the closest one
                # Have to set it here because we need to know how long the times array is
                dynamic_object_positions = []
                for obj in dynamic_objects:
                    obj_times, obj_points = obj.get_traj(new_occ)
                    aligned_trajectory = []
                    current_index = 0
                    for t_index in range(len(times)):
                        # Finds the position at the earliest time latest or equal to the current
                        while obj_times[current_index] < times[t_index] and current_index < len(obj_times) - 1:
                            current_index += 1
                        if current_index < len(obj_times) - 1:
                            aligned_trajectory.append(obj_points[current_index])
                        else:
                            # We assume the object disappears after its trajectory is done
                            aligned_trajectory.append(None)
                    dynamic_object_positions.append(aligned_trajectory)
            # Loop through all objects and all times, check if they come too close
            # If any are too close, fail and recreate path from before
            # NOTE: This assumes that the last object index is not important basically
            fail = False
            fail_index = 0
            for t_index in range(len(times)):
                current_pos = points[t_index]
                for obj_i, obj_trajectory in enumerate(dynamic_object_positions):
                    if obj_trajectory[t_index] is None:
                        # If object is gone, continue
                        continue
                    distsq = (obj_trajectory[t_index][0] - current_pos[0])**2 + (obj_trajectory[t_index][1] - current_pos[1])**2
                    if distsq <= dynamic_objects[obj_i].required_distancesq:
                        fail = True
                        fail_index = t_index
                        break
                if fail:
                    break
            if fail_index > longest_length:
                longest_length = fail_index
                longest_path = tree, times, points
            if fail:
                continue
            # If no collision, return
            return True, new_occ, tree, times, points
        tree = longest_path[0]
        times = longest_path[1]
        points = longest_path[2]
        if fail_index > 0:
            curr_pos = points[longest_length - 1]
            _, tree = self._prune_tree(tree, start_pos, curr_pos)
        return False, new_occ, tree, times[:longest_length], points[:longest_length]

def double_array(arr):
    # Makes each thing in a matrix 2x2
    result = []
    for row in arr:
        result_row = []
        for item in row:
            result_row.append(item)
            result_row.append(item)
        result.append(result_row)
        result.append(result_row)
    return result

if __name__ == "__main__":
    #random.seed(1)
    p = Planner(6, 1, 1)

    #occ = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    occ = np.array([[0, 1], [0, 0]])
    occ = double_array(occ)
    start_pos = (0, 0)

    # Uncomment to use standard 6x6 template
    #occ = np.array([[0, 0, 0, 1, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 1, 1], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
    #occ = double_array(occ)
    #start_pos = (0, 0)

    #human = Human((0, 0), (0, 8), 1)
    #human_traj = human.get_traj(occ, dt=10)

    projectile = Projectile((3, 3), (0, 0))

    custom = Custom(([0, 1, 2], [(2, 1), (1, 1), (0, 1)]), 0.9)

    spos = (1.9, 1.9)
    static = Custom(([0, 100000, 100001], [spos, spos, spos]), 1)

    # Uncomment to generate random occupancy
    #prob = 0.9
    #s = 6
    #occ = np.random.choice([0, 1], size=(s, s), p=[prob, 1 - prob])
    #occ[0][0] = 0
    #occ = double_array(occ)
    #start_pos = (0, 0)

    # Edge cases
    #occ = np.array([[1, 0, 1], [0, 0, 0], [1, 0, 1]])
    #occ = double_array(occ)
    #start_pos = (0, 2)

    #random.seed(1)
    #o = []
    #success1, occ1, tree1, pt, pp = p.generate_compatible_plan(occ, o, start_pos)
    #index = 50
    #success2, occ2, tree2, pt2, pp2 = p.generate_compatible_plan(occ1, o, start_pos, current_pos=pp[index], current_time=pt[index], previous_tree=tree1)
    #print(occ1)
    #print(occ2)
    #p.visualize_plan(occ1, o, (pt, pp), t_step=1)
    #p.visualize_plan(occ2, o, (pt2, pp2), t_step=1)

    objects = [static]
    success, new_occ, tree, plan_times, plan_points = p.generate_compatible_plan(occ, objects, start_pos)
    print("Success", success)
    p.visualize_plan(occ, objects, (plan_times, plan_points), t_step=1)