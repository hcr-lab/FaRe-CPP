from FireBotMAP import Map_generator
import yaml
from FireBotScoutFov import Exploration
import numpy as np
import matplotlib.pyplot as plt
import imageio
import math
pgm_filename = "/home/skachavarapu/home/Research_Track/house_ipa_results/map.pgm"
yaml_filename = "/home/skachavarapu/home/Research_Track/house_ipa_results/map.yaml"
map_generator = Map_generator()
grid_map = map_generator.load_pgm(pgm_filename)

yaml_data = map_generator.load_yaml(yaml_filename)

def fov(grid_map, start_pos, radius, base_angle, fov_angle=140):
    best_grid = None
    max_area = 0
    best_angle = None

    grid = np.copy(grid_map)
    angles = np.deg2rad(np.linspace(base_angle - fov_angle / 2, base_angle + fov_angle / 2, num=400))
    y_start, x_start = start_pos  # Start position interpreted as (y, x)

    for angle in angles:
        for r in range(1, radius + 1):
            x = int(x_start + r * np.cos(angle))
            y = int(y_start + r * np.sin(angle))
            if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
                if grid[y, x] == 0:  # Occupied, block visibility
                    break
                elif grid[y, x] == 254:  # Unoccupied, update to visible
                    grid[y, x] = 150
            else:
                break

    explored_area = np.sum(grid == 150)
    if explored_area > max_area:
        max_area = explored_area
        best_grid = grid
        best_angle = base_angle

    return best_grid, best_angle
           
def find_frontier_cells( grid_map, explored_value, unexplored_value):
    
    rows, cols = grid_map.shape
 
    frontier_cells = []

    for i in range(rows):
        for j in range(cols):

            if grid_map[i, j] == unexplored_value:

                if ((i > 0 and grid_map[i-1, j] == explored_value) or
                    (i < rows - 1 and grid_map[i+1, j] == explored_value) or
                    (j > 0 and grid_map[i, j-1] == explored_value) or
                    (j < cols - 1 and grid_map[i, j+1] == explored_value)):

                    frontier_cells.append((i, j))

    return frontier_cells
file_path = '/home/skachavarapu/home/Research_Track/TPSA/warehouse/fov_90_new/wp_ori_data.txt'
with open(file_path, 'r') as f:
    lines = f.readlines()
    wp = eval(lines[0].split('=')[1].strip())
    ori = eval(lines[1].split('=')[1].strip())

radius = 100
angles = [rad * 180 / math.pi for rad in ori]
start_pos = wp
explored_value = 150  
unexplored_value = 254  

if len(angles) == len(start_pos):
    for i in range(len(angles)):
        grid_map , angle = fov(grid_map, start_pos[i], radius,angles[i])
        


explored_value = 150  
unexplored_value = 254  



print (angle)
plt.imshow(grid_map, cmap='gray', vmin=0, vmax=255)
plt.colorbar(label='Cell Value')
plt.title('Grid Map after FoV Update')
plt.savefig('area')
plt.show()
imageio.imwrite('area.pgm', grid_map.astype(np.uint8))

