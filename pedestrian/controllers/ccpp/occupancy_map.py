import numpy as np
import math
from utils import get_grid_index_at_pos

def generate_occupancy_map(map_size, dim, supervisor_node):
    # Takes in root node of the world
    # Finds all cube like things and their bounding boxes
    # Returns an int 2D array (1 or 0) corresponding to whether an object is in that location
    # Discretizes with the grid size equal to the robot size
    
    # empty occupancy map (0 = free, 1 = occupied)
    oc_map = np.zeros(shape=(dim,dim))
    
    # get the group containing all walls
    group_node = supervisor_node.getFromDef("STATIC")
    # get the field of the group containg the children (=wall) nodes
    children_field = group_node.getField('children')
    # how many children does it have?
    nb_of_walls = children_field.getCount()
    
    for i in range(nb_of_walls):
        wall = children_field.getMFNode(i)
        
        position = wall.getPosition()
        size = wall.getField('children').getMFNode(0).getField('geometry').getSFNode().getField('size').getSFVec3f()
        
        add_rectangle_to_occupancy(size, position, oc_map, map_size, dim)
    return oc_map

def add_rectangle_to_occupancy(size, position, oc_map, map_size, dim):
    # for now assume:
        # retangles are not rotated other than 90 degrees
        # only rectangles
        
    # start with top left corner
    size_in_x = size[0]
    size_in_y = size[1]
    current_x = position[0] - size_in_x / 2
    current_y = position[1] - size_in_y / 2
    
    
    step_value = map_size/dim
    
    while current_x < position[0] + size_in_x / 2:
        while current_y < position[1] + size_in_y / 2:
            grid = get_grid_index_at_pos(current_x, current_y, map_size, dim)
            oc_map[grid[0]][grid[1]] = 1
            current_y += step_value
        current_x += step_value
        current_y = position[1] - size_in_y / 2
    return
