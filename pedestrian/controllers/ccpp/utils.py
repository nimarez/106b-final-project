import math

def get_grid_index_at_pos(x, y, map_size, dim):
    return (math.floor(((x + map_size / 2)/map_size)*dim), math.floor(((-y + map_size / 2)/map_size)*dim))

def get_pos_at_grid_index(i, j, map_size, dim):
    return (round(i * map_size / dim - map_size / 2, 3), -round(j * map_size / dim - map_size / 2, 3))

def get_vel_in_world_co_per_second(grid_per_sec):
    # size of grid = map_size / dim
    return grid_per_sec * (map_size / dim)

def get_vel_in_grid_per_second(world_co_per_sec):
    # size of grid = map_size / dim
    return world_co_per_sec / (map_size / dim)