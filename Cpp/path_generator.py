import numpy as np
import matplotlib.pyplot as plt
import yaml
from PIL import Image
from FireBot_path_optimizer import WayPointOptimizer

import csv

def revisit_time_(linear_length, cumulative_rotation, linear_speed = 0.3, rotational_speed= 0.52):
    
    t_l = linear_length / linear_speed
    
    
    t_r = cumulative_rotation / rotational_speed
    
    
    t_t = t_l + t_r
    
    
    
    return t_t


def load_pgm(pgm_path):
        with Image.open(pgm_path) as img:
            return np.array(img)

def load_yaml(yaml_file_path):
    """Load YAML file and return the content."""
    with open(yaml_file_path, 'r') as file:
        return yaml.safe_load(file)

def heuristic(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(map_data, start, goal, free_space=[254]):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-way connectivity
    open_set = {start}
    came_from = {}
    
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, np.inf))
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)  # Optional: include start in path
            return path[::-1]  # Return reversed path
        
        open_set.remove(current)
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check if within bounds and navigable
            if 0 <= neighbor[0] < map_data.shape[0] and 0 <= neighbor[1] < map_data.shape[1]:
                if map_data[neighbor[0], neighbor[1]] not in free_space:
                    continue
                
                tentative_g_score = g_score[current] + 1  # Cost = 1 per step
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.add(neighbor)
    
    return [] 
    
def convert_pgm_to_binary_custom(image_array):
    """
    Convert a .pgm file to a binary .png file where value 254 is converted to white (255)
    and all other values to black (0).

    Parameters:
    - pgm_file_path: str, path to the .pgm file
    - png_file_path: str, path where the .png file will be saved
    """
    # Load the PGM file
   

    # Convert value 254 to white (255), and all other values to black (0)
    binary_image_array = np.where(image_array == 254, 255, 0).astype(np.uint8)

    # Convert the modified NumPy array back to an image
    binary_image = Image.fromarray(binary_image_array)    
    return binary_image
def rotate_array_90_degrees(array):
    # Rotate the array 90 degrees clockwise
    return np.rot90(array, 1) 
def flip_array_vertically(array):
    # Flip the array vertically
    return np.flipud(array)   
    
#map_data = load_pgm("/home/skachavarapu/home/Research_Track/house_ipa_results/map/map.pgm")
#metadata = load_yaml("/home/skachavarapu/home/Research_Track/house_ipa_results/map/map.yaml")
#map_data = rotate_array_90_degrees(map_data)
#map_data = flip_array_vertically(map_data)
#print(np.unique(map_data))
#resolution = metadata['resolution']
#origin = metadata['origin'][:2]  # Only need x and y

# Transform goal coordinates (these are placeholders, replace with actual transformation based on map)
goals_transformed = [
    # Placeholder transformation; replace with the actual transformation logic based on map's resolution and origin
]
#goals1 = [(350, 67), (315, 102), (311, 102), (295, 108), (310, 152), (317, 152), (327, 154), (343, 247), (331, 251), (308, 297), (320, 335), (310, 373), (300, 371), (287, 420), (225, 431), (233, 408), (198, 384), (229, 349), (274, 379), (271, 337), (270, 331), (274, 313), (231, 299), (162, 330), (153, 290), (145, 215), (183, 228), (185, 228), (193, 258), (208, 254), (242, 265), (259, 215), (234, 210), (222, 185), (273, 177), (269, 164), (253, 129), (219, 98), (207, 68), (278, 77), (350, 67)]

def calculate_and_plot_path(map_data, goals, best_path, resolution, cummulative_rotation, output_dir, before_wpo=True, after_wpo=True):
    total_path_length = 0
    total_path_length1 = 0

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(map_data, cmap='gray')
    
    # Copy grids for before and after WPO
    grid1 = np.copy(map_data)
    grid2 = np.copy(map_data)

    # Calculate and plot path before WPO
    if before_wpo:
        for i in range(len(goals) - 1):
            path = a_star(grid1, goals[i], goals[i+1])
            if path:
                total_path_length += len(path) * resolution
                y, x = zip(*path)
                ax.plot(x, y, color='dimgray', alpha=0.5)
    
    # Calculate and plot path after WPO
    if after_wpo:
        for i in range(len(best_path) - 1):
            path1 = a_star(grid2, best_path[i], best_path[i+1])
            if path1:
                total_path_length1 += len(path1) * resolution
                y1, x1 = zip(*path1)
                ax.plot(x1, y1, color='r')

    # Plotting goals
    for goal in goals:
        ax.plot(goal[1], goal[0], 'b*', markersize=3)

    # Calculate revisit time
    t_t = revisit_time_(total_path_length1, cummulative_rotation, linear_speed=0.3, rotational_speed=0.52)
    print(f'path_length: {total_path_length1}, cummulative rotations: {cummulative_rotation}, revisit_time: {t_t}')
    
    # Save metrics to CSV
    metrics = ['path_length', total_path_length1, 'cummulative rotations', cummulative_rotation, 'revisit_time', t_t]
    csv_file = f'{output_dir}/metrics.csv'
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(metrics)
    
    # Save the plot
    plt.legend()
    plt.savefig(f'{output_dir}/path.png', dpi=200)
    plt.show()
    return metrics
#before_wpo = True
#after_wpo = True
#goals1 = best_path
#total_path_length = 0
#total_path_length1 = 0
#fig, ax = plt.subplots(figsize=(10, 10))
#b_map = convert_pgm_to_binary_custom(map_data)
#cummulative_rotation = 574
#area_map = load_pgm('area.pgm')
#ax.imshow(map_data, cmap='gray')
#grid1 = np.copy(map_data)
#grid2= np.copy(map_data)

#if before_wpo:
    #for i in range(len(goals) - 1):
     #   path = a_star(grid1, goals[i], goals[i+1])
      #  if path:
            
       #     total_path_length += len(path) * resolution
            
        #    y, x = zip(*path)
         #   ax.plot(x, y, color='dimgray', alpha = 0.5)  
#if after_wpo:
 #   for i in range(len(goals1) - 1):
  #      path1 = a_star(grid2, goals1[i], goals1[i+1])
   #     if path1:
    #        
     #       total_path_length1 += len(path1) * resolution
            
      #      y1, x1 = zip(*path1)
       #     ax.plot(x1, y1, color='r')


# Plotting goals
#for goal in goals:
   # ax.plot(goal[1], goal[0], 'b*', markersize=3)
 
#t_t = revisit_time_(total_path_length1, cummulative_rotation, linear_speed = 0.3, rotational_speed= 0.52)
#print(f'path_length {total_path_length1},cummulative rotations {cummulative_rotation}, revisit_time {t_t}')
#metrics = ['path_length',total_path_length1,'cummulative rotations',cummulative_rotation, 'revisit_time' ,t_t]
#csv_file = 'latest_results_3/ipa.csv'
#with open(csv_file,'w',newline = '') as file:
 #   writer = csv.writer(file)
  #  writer.writerow(metrics)
#plt.legend()
#plt.savefig('latest_results_3/ipa.png', dpi=200) 
#plt.show()




