class Planner(object):
    def __init__(self):
        self.curr_plan

    def generate_occupancy_map(self, root_node):
        # Takes in root node of the world
        # Finds all cube like things and their bounding boxes
        # Returns an int 2D array (1 or 0) corresponding to whether an object is in that location
        # Discretizes with the grid size equal to the robot size
        pass

    def generate_plan(self, extended_occupancy_map, dynamic_objects, curr_plan):
        # Extended occupancy map is same as occupancy map but with the potential for a square to be already explored
        # Dynamic objects is a representation of human intent, e.g. list of dict of time to coordinate
        # Returns an array of squares to visit indexed by time. 
        # Thing to consider: we must complete the other side of squares we already visited first, if we decide to replan
        # - Solution: Replan given the current completed prefix of the plan, record big squares as being fully cleaned, partially cleaned, etc.
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