import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml

class Map_generator():
    def __init__(self):
        pass
    
    def load_pgm(self,pgm_path):
        with Image.open(pgm_path) as img:
            return np.array(img)

    def load_yaml(self,yaml_path):
        
        with open(yaml_path ,'r') as file:
            return yaml.safe_load(file)

    def convert_map_to_rgb(self,grid_map, frontiers):
        rgb_map = np.zeros((grid_map.shape[0], grid_map.shape[1], 3), dtype=np.uint8)
        rgb_map[grid_map == 0] = [0, 0, 255]  
        rgb_map[grid_map == 205] = [255,255,255]  
        rgb_map[grid_map == 254] = [0, 255, 0]
        if frontiers == True:
            rgb_map[grid_map == 255] == [51, 68, 255]
        return rgb_map

    def plot_rgb_map(self,grid_map,goals = None ,title = 'rgb_map' ,frontiers=False, save_fig = False):
        
        rgb_map = self.convert_map_to_rgb(grid_map, frontiers = frontiers )
        plt.imshow(rgb_map)
        if goals is not None:
            plt.scatter(goals[1], goals[0], color='red', s = 3)
            
        plt.title(title)
        if save_fig != False:
           plt.savefig(f'/Users/sriapple/research_tarck/grid_maps/{save_fig}_iteration.png')
        plt.show()
    def plot_iterations(self,goals):
        for i in range(len(goals)):
            iteration = goals[i]['iteration']
            goal = goals[i]['goals']['goal']
            graph = goals[i]['graph']
            
            # Use frontiers from the next iteration if it exists
            if i < len(goals) - 1:
                frontiers = goals[i + 1]['frontiers']
            else:
                frontiers = goals[i]['frontiers'] 
            
            
            title = f'Iteration {iteration} - Goal: {goal}'
            start_pos = goal
            
            
            plt.figure()
            plt.imshow(graph, cmap='gray', interpolation='nearest')
            plt.colorbar(label='Cell Value')
            
            plt.scatter(start_pos[1], start_pos[0], color='blue', s=0.1, marker='o', label='Start Pose')
            if frontiers:
                frontier_x, frontier_y = zip(*frontiers)
                plt.scatter(frontier_y, frontier_x, color='red', marker='o', s=0.1, label='Frontier Cells')
            
            plt.title(title)
            plt.legend()
            
            # Save the figure for each iteration
            plt.savefig(f'iteration_{iteration}_goal_{goal}.png')
            plt.close()
    def plot_frontiers(self,grid_map,frontiers):
        plt.imshow(grid_map, cmap='gray')

      
        plt.scatter(frontiers[:, 1], frontiers[:, 0], color='red', s=0.5)  

        plt.title('Frontiers on Grid Map')
        plt.show()

    def estimate_area(self,grid_map, yaml_data,state):
   
        unoccupied_cells = np.sum(grid_map == state)
        resolution = yaml_data.get("resolution", 1)
        area_per_cell = resolution ** 2
        return unoccupied_cells * area_per_cell
