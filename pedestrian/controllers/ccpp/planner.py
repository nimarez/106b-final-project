import matplotlib.pyplot as plt
import numpy as np
import random
from dynamic_objects import DynamicObject, Human, Projectile


class Planner(object):
    def __init__(self, dim):
        # Dim is a tuple for size of the world
        self.dim = dim
        # Up left down right
        self.coords_to_check = [(0, -1), (-1, 0), (0, 1), (1, 0)]

    def visualize_plan(self, extended_occupancy_map, dynamic_objects, plan, t_step=4):
        plt.matshow(extended_occupancy_map)
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
                if i % t_step == 0:
                    plt.text(xitem, yitem, str(t), color=textcolor)
            is_plan = False
        plt.gca().set_xticks([x - 0.5 for x in plt.gca().get_xticks()][1:], minor='true')
        plt.gca().set_yticks([y - 0.5 for y in plt.gca().get_yticks()][1:], minor='true')
        plt.grid(which='minor')
        plt.show()

    def generate_occupancy_map(self, root_node):
        # Takes in root node of the world
        # Finds all cube like things and their bounding boxes
        # Returns an int 2D array (1 or 0) corresponding to whether an object is in that location
        # Discretizes with the grid size equal to the robot size
        pass

    def _generate_spanning_tree(self, large_map, start_pos, previous_tree=None):
        """
        Generates a (not minimal) spanning tree, returns a dictionary of (pos) --> [boolean for up down left right if it connects]
        large_map: Returned from _largify_cells, binary matrix for large cells for spanning tree algorithm
        start_pos: tuple of (x, y) starting position in large coordinates
        NOTE: This is not deterministic, we can generate random trees each time
        NOTE: Edges are only one way, from parent to child
        """
        # Previous tree should just be a line -- assert this before
        if previous_tree is not None:
            curr, curr_val = start_pos, previous_tree[start_pos]
            while any(curr_val):
                index = curr_val.index(True)
                curr = (curr[0] + self.coords_to_check[index], curr[1] + self.coords_to_check[index])
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
        Cuts off the edges if odd-sized, and marks partial occupancy as full occupancy
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
                row.append(int(not(np.all(np.array([s1, s2, s3, s4]) == np.zeros(4)))))
            result.append(row)
        return result

    def _prune_tree(self, tree, current_position):
        """
        tree: Returned from generated spanning tree
        current_position: Current location in tree
        Prunes the tree to only contain nodes which are partially visited
        This is useful because we can mark the fully visited nodes as obstacles essentially
            It costs nothing to go around them because the paths are all uniform length because space-filling
        Also removes the unvisited squares, because we want to regenerate the path if we encounter a dynamic obstacle 
        Returns pruned tree -- this should be a line
        """
        # TODO
        # NOTE: Assumes that walk tree is deterministic (!!)
        path = self._walk_tree(tree, start_pos)
        points = path.values()
        pass

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
                # TODO: This might fail when we have a tree of size 1 because previous is never set
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

    def generate_plan(self, extended_occupancy_map, start_pos, current_pos=None, previous_tree=None, dt=0.5):
        """
        Generates a path (dict of time to position) through an occupancy map
        Extended occupancy map is same as occupancy map but with the potential for a square to be already explored
        Dynamic objects is a representation of human intent, e.g. list of dict of time to coordinate
        When starting out, current_pos and previous_tree are not needed
            Can set these to regenerate part of the path
        Thing to consider: we must complete the other side of squares we already visited first, if we decide to replan
        - Solution: Replan given the current completed prefix of the plan,
            record big squares as being fully cleaned, partially cleaned, etc.
        Future work:
        - Make spanning tree thing work for obstacles which only cover 1/4 of the mega cell
        """
        pruned_tree = None
        if current_pos is not None:
            # TODO: Update occupancy map with fully occupied stuff from previous run, should probably be done outside this function
            pruned_tree = self._prune_tree(previous_tree, current_pos)
        large_cells = self._largify_cells(extended_occupancy_map)
        tree = self._generate_spanning_tree(large_cells, start_pos, pruned_tree)
        path = self._walk_tree(tree, start_pos)
        # Assumes the robot moves with constant velocity of 1 square/dt
        times = []
        points = []
        for i in range(len(path)):
            times.append(dt*i)
            points.append(path[i])
        return times, points

    def main(self, root_node, human_handle):
        occ = self.generate_occupancy_map(root_node)
        unexplored = set() # TODO
        timesteps = 10
        while unexplored:
            p = self.generate_plan(occ, self.get_human_plan(human_handle), self.curr_plan) # TODO get human stuff)
            self.follow_plan(p, timesteps, occ, unexplored) # Pass in occupancy map/unexplored to update it

    
    def follow_plan(self, plan, occupany_grid, unexplored):
        # move a timestep
        # if you see a human, do the human stuff and stop this path
        # generate a new plan
        pass

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
    #random.seed(0.41783174376782783)
    p = Planner(6)

    #occ = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    occ = np.array([[0, 0, 0], [0, 0, 0]])
    occ = double_array(occ)
    start_pos = (0, 0)

    # Uncomment to use standard 6x6 template
    #occ = np.array([[0, 0, 0, 1, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 1, 1], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
    #occ = double_array(occ)
    #start_pos = (0, 0)

    #human = Human((0, 0), (0, 8), 1)
    #human_traj = human.get_traj(occ, dt=10)

    projectile = Projectile((3, 3), (1, 0.5))

    # Uncomment to generate random occupancy
    prob = 0.95
    occ = np.random.choice([0, 1], size=(30, 30), p=[prob, 1 - prob])
    occ[0][0] = 0
    occ = double_array(occ)
    start_pos = (0, 0)

    # Edge cases
    #occ = np.array([[1, 0, 1], [0, 0, 0], [1, 0, 1]])
    #occ = double_array(occ)
    #start_pos = (0, 2)

    plan = p.generate_plan(occ, start_pos)
    p.visualize_plan(occ, [], plan, t_step=100000)