import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml
import time
from IPython.display import clear_output
from scipy.spatial.distance import pdist, squareform
from itertools import permutations
from FireBotMAP import Map_generator

map_generator = Map_generator()

class Scout:
    def __init__(self):
        pass

    def get_circle_perimeter_points(self, center, radius, num_points=360):
        perimeter_points = set()
        for theta in np.linspace(0, 2 * np.pi, num=num_points):
            x = int(radius * np.cos(theta) + center[0])
            y = int(radius * np.sin(theta) + center[1])
            perimeter_points.add((x, y))
        return list(perimeter_points)
    
    def line_of_sight(self, grid_map, start, end, unoccupied_value):
      
        x0, y0 = start
        x1, y1 = end
        #print(x1,y1)
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  

        while True:
            if x0 == x1 and y0 == y1:
                return True
            if grid_map[x0, y0] != unoccupied_value:
                return False
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return (x1,y1)

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

    
    def increment_path_to_frontiers(self, grid_map, start_position, frontiers):
        """Increment cells along the path from start position to each frontier cell."""
        updated_map = np.copy(grid_map)  

        for frontier in frontiers:
            self.increment_path(updated_map, start_position, frontier)
        
        return updated_map

    def increment_path(self, grid_map, start, end):
        """Increment cells along the line from start to end point."""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  

        while True:
            grid_map[x0, y0] += 1 
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
    def increment_cells_within_circle(self, grid_map, start_position, radius, unoccupied_value):
        
        
        assert grid_map[start_position] == unoccupied_value, "Starting position must be in unoccupied space."
        
        updated_map = np.copy(grid_map) 
        rows, cols = grid_map.shape

        for i in range(rows):
            for j in range(cols):
                if self.is_within_circle(start_position, (i, j), radius):
                    if grid_map[i, j] == unoccupied_value and self.line_of_sight(grid_map, start_position, (i, j), unoccupied_value):
                        if grid_map[i, j] != 255:
                            updated_map[i, j] += 1
        
        return updated_map


    def is_within_circle(self, center, point, radius):
        return np.linalg.norm(np.array(center) - np.array(point)) <= radius
    
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
       
        for i, frontier in enumerate(frontiers):
            sub_graph = self.scout.increment_cells_within_circle(graph, frontier, self.surveillance_range, self.free_cells)
            a = map_generator.estimate_area(sub_graph, self.yaml_data, self.state)  # Assuming map_generator is defined
            total_area =  a
            explored_area.append({
                'id': i,
                'frontier': frontier,
                'area': total_area,
                'sub_graph': sub_graph
            })
        max_area_dict = max(explored_area, key=lambda x: x['area'])
        selected_frontier, area, graph = max_area_dict['frontier'], max_area_dict['area'], max_area_dict['sub_graph']
        return selected_frontier, area, graph

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
            selected_frontier, area, updated_graph = self.surveillance(iteration, frontiers, graph, total_area)
            total_area = area 
            iteration += 1 
            graph = updated_graph
            end_time = time.time()
            area_goals.append({'iteration': iteration, 'goals': {'goal': selected_frontier, 'area': area}, 'graph':graph})
            
            print(f'iteration: {iteration} goal : {selected_frontier}   explored area : {area} frontiers: {len(frontiers)} excution_time: {int(end_time - start_time)} seconds ' )
            
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