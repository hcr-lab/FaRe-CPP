from FireBotMAP import Map_generator
import yaml
from FireBotScout_GPU import Exploration
import numpy as np

def main():
    map_generator = Map_generator()

    starting_position = [(350, 67)]
    explored_value = 255 
    unexplored_value = 254 
    state = explored_value 
    steps = 3
    frontier_drop_rate = 0
    pgm_filename = r"D:\srinika\Research_Track\house_ipa_results\map\map.pgm"
    yaml_filename = r"D:\srinika\Research_Track\house_ipa_results\map\map.yaml"


    grid_map = map_generator.load_pgm(pgm_filename)
    yaml_data = map_generator.load_yaml(yaml_filename)
    print(np.unique(grid_map))
    print(map_generator.estimate_area(grid_map,yaml_data,254))

    map_generator.plot_rgb_map(grid_map)

    explore = Exploration(grid_map, 40, unexplored_value, state, yaml_data)

    goals = explore.set_goals(starting_position, explored_value, unexplored_value, steps,frontier_drop_rate)
    for item in goals:
        iteration = item['iteration']
        goal = item['goals']['goal']
        graph = item['graph']  
        title = f'Iteration {iteration} - Goal: {goal} - Area: {item["goals"]["area"]}'
        
        map_generator.plot_rgb_map(graph, goal, title, frontiers =True) #save_fig = iteration )

if __name__ == '__main__':
    main()