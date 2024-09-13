import numpy as np
import time
import random
import multiprocessing
from scipy.spatial.distance import pdist, squareform
from itertools import permutations
from FireBotMAP import Map_generator
import matplotlib.pyplot as plt
map_generator = Map_generator()



class Scout:
    def __init__(self):
        pass

    def fov(self, grid_map, start_pos, radius, fov_angle=160):
        
        base_angles = [0, 180, 270, 90]  # Angles for N, S, W, E
        best_grid = None
        max_area = 0
        best_angle = None

        for base_angle in base_angles:
            grid = np.copy(grid_map)
            angles = np.deg2rad(np.linspace(base_angle - fov_angle / 2, base_angle + fov_angle / 2, num=100))
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
    

    

    def find_frontier_cells(self, grid_map, explored_value, unexplored_value):
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


class Exploration:
    def __init__(self, grid_map, surveillance_range, free_cells, state, yaml_data):
        self.grid_map = grid_map
        self.surveillance_range = surveillance_range
        self.free_cells = free_cells
        self.state = state
        self.yaml_data = yaml_data
        self.scout = Scout()  
       
        
    def surveillance(self, iteration, frontiers, graph, area):
        explored_area = []
        #print(frontiers)
        for i, frontier in enumerate(frontiers):
            sub_graph, angle = self.scout.fov(graph, frontier, self.surveillance_range)
            #print(sub_graph)
            #map_generator.plot_rgb_map(sub_graph)
                
            a = map_generator.estimate_area(sub_graph, self.yaml_data, self.state)  
            total_area =  a
            explored_area.append({
                'id': i,
                'frontier': frontier,
                'area': total_area,
                'sub_graph': sub_graph,
                'orientation': angle
            })
        max_area_dict = max(explored_area, key=lambda x: x['area'])
        selected_frontier, area, graph ,orientation = max_area_dict['frontier'], max_area_dict['area'], max_area_dict['sub_graph'] , max_area_dict['orientation']
        return selected_frontier, area, graph,orientation

    def set_goals(self, current_pos, explored_value, unexplored_value,steps):
        total_area = 0
        iteration = 0
        graph = self.grid_map
        area_goals = []

        for i in range(steps):
            start_time = time.time()
            
            frontiers = current_pos if i == 0 else self.scout.find_frontier_cells(graph, explored_value, unexplored_value)
            if not frontiers:
                print("No more frontiers found. Stopping exploration.")
                break
            selected_frontier, area, updated_graph, orientation = self.surveillance(iteration, frontiers, graph, total_area)
            total_area = area 
            iteration += 1 
            graph = updated_graph
            end_time = time.time()
            area_goals.append({'iteration': iteration, 'goals': {'goal': selected_frontier,'orientation':orientation, 'area': area}, 'graph':graph})
            
            print(f'iteration: {iteration} goal : {selected_frontier, orientation}   explored area : {area} frontiers: {len(frontiers)} excution_time: {int(end_time - start_time)} seconds ' )
            
        return area_goals
    def optimize_goals(self,goal_points):
   
        points = [item['goals']['goal'] for item in goal_points]

        
        distance_matrix = squareform(pdist(points, 'euclidean'))

        
        def total_distance(path):
            return sum(distance_matrix[path[i], path[i+1]] for i in range(len(path) - 1))

        
        all_paths = permutations(range(1, len(points)))
        min_path = None
        min_distance = float('inf')

       
        for path in all_paths:
            current_path = (0,) + path + (0,)
            current_distance = total_distance(current_path)
            if current_distance < min_distance:
                min_distance = current_distance
                min_path = current_path

        
        min_path_points = [points[i] for i in min_path]

      
        coord_to_iter_index = {item['goals']['goal']: idx for idx, item in enumerate(goal_points)}

       
        ordered_goals = [goal_points[coord_to_iter_index[point]] for point in min_path_points[1:-1]]

        return min_path_points, min_distance, ordered_goals
