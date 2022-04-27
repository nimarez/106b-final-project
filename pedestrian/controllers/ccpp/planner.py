import matplotlib.pyplot as plt
import numpy as np

class Planner(object):
    def __init__(self, dim):
        # Dim is a tuple for size of the world
        self.dim = dim
        # Up left down right
        self.coords_to_check = [(0, -1), (-1, 0), (0, 1), (1, 0)]

    def visualize_plan(self, extended_occupancy_map, dynamic_objects, plan):
        plt.matshow(extended_occupancy_map)
        if plan is not None:
            for p1, p2 in zip(plan[:-1], plan[1:]):
                x1 = [p1[0], p2[0]]
                y1 = [p1[1], p2[1]]
                plt.plot(x1, y1, color="blue")
        plt.show()

    def generate_occupancy_map(self, root_node):
        # Takes in root node of the world
        # Finds all cube like things and their bounding boxes
        # Returns an int 2D array (1 or 0) corresponding to whether an object is in that location
        # Discretizes with the grid size equal to the robot size
        pass

    def _generate_spanning_tree(self, large_map, start_pos):
        """
        Generates a (not minimal) spanning tree, returns a dictionary of (pos) --> [boolean for up down left right if it connects]
        large_map: Returned from _largify_cells, binary matrix for large cells for spanning tree algorithm
        start_pos: tuple of (x, y) starting position in large coordinates
        NOTE: This should probably be improved to make the tree as straight as possible, but that's effort
        """
        to_visit = [start_pos]
        edges = {}
        visited = set()
        while to_visit:
            current = to_visit.pop()
            if current in visited:
                continue
            visited.add(current)
            current_edges = [False, False, False, False]
            for i, (xdiff, ydiff) in enumerate(self.coords_to_check):
                newx, newy = current[0] + xdiff, current[1] + ydiff
                if newx < 0 or newy < 0 or newx >= len(large_map[0]) or newy >= len(large_map) or large_map[newy][newx] == 1:
                    continue
                to_visit.append((newx, newy))
                current_edges[i] = True
            edges[current] = current_edges
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

    def _walk_tree(self, tree, start_pos_large):
        """
        Takes in a tree (in large coordinates) and start_pos (also in large coordinates !!) and produces a path from walking that tree
        TODO: Handle prefix
        """
        stack = [start_pos_large]
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
            (0, 0): 0,
            (0, 1): 1, 
            (1, 1): 2,
            (1, 0): 3
        }
        # Always start in the top right so we can go counterclockwise
        path = [(start_pos_large[0] * 2 + 1, start_pos_large[1] * 2)]
        while stack:
            current = stack.pop()
            # Don't check for visited here because we're probably already visited
            visited.add(current)
            current_position = path[-1]
            xmod, ymod = current_position[0] % 2, current_position[1] % 2
            shift = coord_check_shift[(xmod, ymod)]
            shifted_coords = self.coords_to_check[shift:] + self.coords_to_check[:shift]
            print("Shift", shift)
            print(shifted_coords)
            print(self.coords_to_check)
            for unshifted_i, (xdiff, ydiff) in enumerate(shifted_coords):
                index = (unshifted_i + shift) % 4
                new_pos = (current[0] + xdiff, current[1] + ydiff)
                if tree[current][index] and new_pos not in visited:
                    # We want to visit current against at the end, and repeat the same thing
                    stack.append(current)
                    stack.append(new_pos)
                    break
            if previous is not None:
                # TODO: This might fail when we have a tree of size 1 because previous is never set
                # Finds the counterclockwise shortest path from the last position to the closest position in the new square
                dx, dy = next_map_right_turn[(xmod, ymod)]
                right_turn_pos = (current_position[0] + dx, current_position[1] + dy)
                if right_turn_pos[0] // 2 == current[0] and right_turn_pos[1] // 2 == current[1]:
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
            previous = current
        return path

    def _generate_spanning_tree_path(self, extended_occupancy_map, prefix):
        # Prefix should just be start pos for now
        if len(prefix) != 1:
            raise Exception("Todo")
        start_pos = prefix[-1]
        large_cells = self._largify_cells(extended_occupancy_map)
        tree = self._generate_spanning_tree(large_cells, start_pos)
        start_pos_big = (start_pos[0] // 2, start_pos[1] // 2)
        # We append start pos because we migth not start out at the intended location on the big square
        return [start_pos] + self._walk_tree(tree, start_pos_big)

    def generate_plan(self, extended_occupancy_map, dynamic_objects, curr_plan):
        # Extended occupancy map is same as occupancy map but with the potential for a square to be already explored
        # Dynamic objects is a representation of human intent, e.g. list of dict of time to coordinate
        # Returns an array of squares to visit indexed by time. 
        # Thing to consider: we must complete the other side of squares we already visited first, if we decide to replan
        # - Solution: Replan given the current completed prefix of the plan, record big squares as being fully cleaned, partially cleaned, etc.
        # Future work:
        # - Make spanning tree thing work for obstacles which only cover 1/4 of the mega cell
        pass

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
    p = Planner(6)
    #occ = np.array([[0, 0, 0, 1, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 1, 1], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
    prob = 0.95
    occ = np.random.choice([0, 1], size=(30, 30), p=[prob, 1 - prob])
    occ[0][0] = 0
    #occ = double_array(occ)
    #occ = double_array([[0, 0], [1, 0]])
    start_pos = (0, 0)
    #plan = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
    plan = p._generate_spanning_tree_path(occ, [start_pos])
    p.visualize_plan(occ, [], plan)