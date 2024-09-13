import tensorflow as tf
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
from numba import cuda


class Exploration:
    def __init__(self,grid_map, surveillance_range, free_cells, state, yaml_data):
        self.grid_map = grid_map
        self.surveillance_range = surveillance_range
        self.free_cells = free_cells
        self.state = state
        self.yaml_data = yaml_data
        self.resolution =  yaml_data.get("resolution", 1)
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
    def line_of_sight(self, grid_map, start, end, unoccupied_value):
        x0, y0 = start
        x1, y1 = end
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
                return True
        

                
    
    def increment_cells_within_circle_tf(self, grid_map, start_position, radius, unoccupied_value):
        grid_map_tensor = tf.constant(grid_map, dtype=tf.float32)
        rows, cols = grid_map.shape
        x, y = tf.meshgrid(tf.range(cols, dtype=tf.float32), tf.range(rows, dtype=tf.float32))

        distances = tf.sqrt(tf.square(x - start_position[1]) + tf.square(y - start_position[0]))
        mask_within_circle = distances <= radius

        increment_mask = tf.zeros_like(grid_map_tensor, dtype=tf.bool)


        def check_line_of_sight(cell):
            end = (tf.cast(cell[0], tf.int32), tf.cast(cell[1], tf.int32))
            return self.line_of_sight(grid_map, start_position, end, unoccupied_value)

        cells_within_circle = tf.where(mask_within_circle)
        #increment_mask = tf.map_fn(check_line_of_sight, cells_within_circle, fn_output_signature=tf.bool)
        increment_mask = tf.scatter_nd(cells_within_circle, increment_mask, [rows, cols])

        # Apply increments where the increment mask is True
        incremented_map = tf.where(increment_mask, grid_map_tensor + 1, grid_map_tensor)

        return incremented_map.numpy()
        
        
        

        

    def estimate_area_tf(self, grid_map, state):
        grid_map_tensor = tf.constant(grid_map, dtype=tf.float32)
        unoccupied_cells = tf.reduce_sum(tf.cast(tf.equal(grid_map_tensor, state), tf.float32))
        area_per_cell = self.resolution ** 2
        total_area = unoccupied_cells * area_per_cell
        
        return total_area.numpy()
    def is_within_circle(self, center, point, radius):
        return np.linalg.norm(np.array(center) - np.array(point)) <= radius
    
    

    def surveillance(self, iteration, frontiers, grid_map, area):
        explored_area = []
        grid_map_tensor = tf.constant(grid_map, dtype=tf.float32)  # Convert grid_map to a TensorFlow tensor

        for i, frontier in enumerate(frontiers):
            updated_grid_map = self.increment_cells_within_circle_tf(grid_map_tensor.numpy(), frontier, self.surveillance_range, self.free_cells)
            total_area = self.estimate_area_tf(updated_grid_map,  self.state)
            
            explored_area.append({
                'id': i,
                'frontier': frontier,
                'area': total_area,
                'sub_graph': updated_grid_map  # Keeping as a TensorFlow tensor for now
            })
        max_area_dict = max(explored_area, key=lambda x: x['area'])
        
        # Extract the details of the best frontier
        selected_frontier = max_area_dict['frontier']
        max_area = max_area_dict['area']
        updated_grid_map = max_area_dict['sub_graph']

        return selected_frontier, max_area, updated_grid_map
    
    
    def set_goals(self, current_pos, explored_value, unexplored_value,steps, frontier_drop_rate):
        total_area = 0
        iteration = 0
        graph = self.grid_map
        area_goals = []

        for i in range(steps):
            start_time = time.time()
            frontiers = current_pos if i == 0 else self.find_frontier_cells(graph, explored_value, unexplored_value)
            file_path = f"D:\srinika\Research_Track\maps\cpp_house\goals\goal{i}.txt"
            with open(file_path, 'w') as file:
                for goal in frontiers:
                   file.write(f"{goal}\n")
            if frontier_drop_rate > 0:
                frontiers = [item for index, item in enumerate(frontiers) if index == 0 or (index + 1) % frontier_drop_rate == 0]
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