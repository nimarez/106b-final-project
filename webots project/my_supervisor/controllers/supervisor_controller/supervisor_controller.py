from controller import Supervisor
import numpy as np
import math

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance
root_node = robot.getRoot()

i = 0

# number of cells on row / column to divide world into
# assumes: square world
diff = 10

def generate_occupancy_map(root_node):
    # Takes in root node of the world
    # Finds all cube like things and their bounding boxes
    # Returns an int 2D array (1 or 0) corresponding to whether an object is in that location
    # Discretizes with the grid size equal to the robot size
    
    # empty occupancy map (0 = free, 1 = occupied)
    oc_map = np.zeros(shape=(diff,diff))
    
    # get the group containing all walls
    group_node = robot.getFromDef("WALLS")
    # get the field of the group containg the children (=wall) nodes
    children_field = group_node.getField('children')
    # how many children does it have?
    nb_of_walls = children_field.getCount()
    
    # orientations = list()
    
    for i in range(nb_of_walls):
        wall = children_field.getMFNode(i)
        
        position = wall.getPosition()
        size = wall.getField('children').getMFNode(0).getField('geometry').getSFNode().getField('size').getSFVec3f()
        
        add_rectangle_to_occupancy(size, position, oc_map, 10, diff)
        
        #print(get_grid_index_at_pos(position[0], position[1], 10, diff))
        
        # orientations.append(wall.getOrientation())
        
        #size = wall.getField('children').getField("Shape").getField("geometry").getField("size")
        
        #print(size)
    print(oc_map)
        
    #translation_field = wall_node.getField('translation')
    #print(translation_field.)

def add_rectangle_to_occupancy(size, position, oc_map, map_size, diff):
    # for now assume:
        # retangles are not rotated other than 90 degrees
        # only rectangles
        
    # start with top left corner
    current_x = position[0] - size[0]
    current_y = position[1] - size[1]
    
    step_value = math.floor(map_size/diff)
    
    while current_x < position[0] + size[0]:
        while current_y < position[1] + size[1]:
            grid = get_grid_index_at_pos(current_x, current_y, map_size, diff)
            oc_map[grid[0]][grid[1]] = 1
            current_y += step_value
        current_x += step_value
        current_y = position[1] - size[1]
    return
        
        
def get_grid_index_at_pos(x, y, map_size, diff):
    return (math.floor(((x + map_size / 2)/map_size)*diff), math.floor(((y + map_size / 2)/map_size)*diff))
    

generate_occupancy_map(root_node)

while robot.step(TIME_STEP) != -1:
  # [CODE PLACEHOLDER 2]

  i += 1