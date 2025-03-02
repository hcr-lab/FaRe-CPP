import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path as mpltPath
import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml
import time
from IPython.display import clear_output
from scipy.spatial.distance import pdist, squareform
from itertools import permutations
from FireBotMAP import Map_generator
import multiprocessing
from Multi_Processing import process_frontier
import random
import math
map_generator = Map_generator()


from collections import defaultdict

class Scout:
    def __init__(self):
        pass

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate cells along the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        line = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            line.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return line

    def fov(self, grid_map, start_pos, radius, fov_angle=133):
        """ computes fov similar to ipa_coverage planning."""
        y_start, x_start = start_pos
        assert grid_map[y_start, x_start] == 254, "Start position must be in an unoccupied area."
        circle_radius = 28 
        cells_in_circle = []
        for y in range(max(0, y_start - circle_radius), min(grid_map.shape[0], y_start + circle_radius + 1)):
            for x in range(max(0, x_start - circle_radius), min(grid_map.shape[1], x_start + circle_radius + 1)):
                dx = x - x_start
                dy = y - y_start
                if dx**2 + dy**2 <= circle_radius**2:
                    cells_in_circle.append((x, y))
        angle_counts = defaultdict(int)
        angle_step = 6     
        for x, y in cells_in_circle:
            if grid_map[y, x] == 254:  
                dx = x - x_start
                dy = y - y_start
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)
                binned_angle = round(angle_deg / angle_step) * angle_step
                binned_angle %= 360
                angle_counts[binned_angle] += 1    
        best_angle_deg = max(angle_counts, key=lambda k: angle_counts[k], default=0)
        best_angle = math.radians(best_angle_deg)   
        grid = np.copy(grid_map)
        near_distance = 0
        far_distance = 23
        near_width = 14
        far_width = 28

        polygon_rotated = [
            (near_distance, -near_width / 2),
            (near_distance, near_width / 2),
            (far_distance, far_width / 2),
            (far_distance, -far_width / 2),
        ]

        polygon_grid = []
        for (dx, dy) in polygon_rotated:
            x_rot = dx * math.cos(best_angle) - dy * math.sin(best_angle)
            y_rot = dx * math.sin(best_angle) + dy * math.cos(best_angle)
            polygon_grid.append((x_start + x_rot, y_start + y_rot))

        path = mpltPath(polygon_grid)
        min_x = min(p[0] for p in polygon_grid)
        max_x = max(p[0] for p in polygon_grid)
        min_y = min(p[1] for p in polygon_grid)
        max_y = max(p[1] for p in polygon_grid)

  
        for y in range(int(min_y) - 1, int(max_y) + 2):
            for x in range(int(min_x) - 1, int(max_x) + 2):
                if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
                    if path.contains_point((x, y)):
                        line_cells = self.bresenham_line(x_start, y_start, x, y)
                        blocked = False
                        for (cx, cy) in line_cells:
                            if not (0 <= cx < grid.shape[1] and 0 <= cy < grid.shape[0]) or grid[cy, cx] == 0:
                                blocked = True
                                break
                        if not blocked and grid[y, x] == 254:
                            grid[y, x] = 150  # Mark as explored

        explored_cells = np.sum(grid == 150)
        return grid, best_angle

    
    
    def find_frontier_cells(self,grid_map, explored_value, unexplored_value, obstacle_value=0, buffer_distance=2):
        rows, cols = grid_map.shape
        frontier_cells = []

        def is_within_buffer(position):
            for dx in range(-buffer_distance, buffer_distance + 1):
                for dy in range(-buffer_distance, buffer_distance + 1):
                    nx, ny = position[0] + dx, position[1] + dy
                    if 0 <= nx < rows and 0 <= ny < cols:
                        if grid_map[nx, ny] == obstacle_value:
                            return True
            return False

        for i in range(rows):
            for j in range(cols):
                if grid_map[i, j] == unexplored_value:
                    if ((i > 0 and grid_map[i-1, j] == explored_value) or
                        (i < rows - 1 and grid_map[i+1, j] == explored_value) or
                        (j > 0 and grid_map[i, j-1] == explored_value) or
                        (j < cols - 1 and grid_map[i, j+1] == explored_value)):

                        if not is_within_buffer((i, j)):
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
        print('range:', self.surveillance_range )
    def surveillance(self, iteration, frontiers, graph, area):    
        with multiprocessing.Pool() as pool:
            # Prepare arguments for each process
            args = [(graph, frontier, self.scout, self.surveillance_range, self.free_cells, self.yaml_data, self.state) for frontier in frontiers]
            # Execute the function in parallel
            results = pool.starmap(process_frontier, args)       
        max_area_dict = max(results, key=lambda x: x['area'])
        selected_frontier, area, graph, ori = max_area_dict['frontier'], max_area_dict['area'], max_area_dict['sub_graph'],max_area_dict['ori']
        return selected_frontier, area, graph,ori
    
    def set_goals(self, current_pos, explored_value, unexplored_value,steps, frontier_drop_rate):
        total_area = 0
        iteration = 0
        t_time = 0
        graph = self.grid_map
        area_goals = []
        for i in range(steps):
            start_time = time.time()
            frontiers = current_pos if i == 0 else self.scout.find_frontier_cells(graph, explored_value, unexplored_value)
            #file_path = f"D:\srinika\Research_Track\maps\cpp_house\goals\goal{i}.txt"

            #with open(file_path, 'r') as file:
                #frontiers = [tuple(map(int, line.strip().strip('()').split(', '))) for line in file]
            if i > 1:
                random.shuffle(frontiers)
            if frontier_drop_rate > 0:
                frontiers = [item for index, item in enumerate(frontiers) if index == 0 or (index + 1) % frontier_drop_rate == 0]
            if not frontiers:
                print("No more frontiers found. Stopping exploration.")
                break
            selected_frontier, area, updated_graph,ori = self.surveillance(iteration, frontiers, graph, total_area)
            total_area = area 
            iteration += 1 
            graph = updated_graph
            end_time = time.time()
            area_goals.append({'iteration': iteration, 'goals': {'goal': selected_frontier, 'area': area,'orientation':ori}, 'graph':graph, 'frontiers':frontiers})
            t_time += end_time - start_time            
            print(f'steps: {iteration} goal : {selected_frontier,ori}   e_area : {int(area)} wp: {len(frontiers)} e_time: {int(end_time - start_time)} seconds t_time = {int(t_time)}  ' )
            
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
