from FireBotMAP import Map_generator
import yaml
from FireBotScout_Multi_Processing import Exploration
from path_generator import calculate_and_plot_path
from FireBot_path_optimizer import WayPointOptimizer
from config import config 
import os
def main():
    map_generator = Map_generator()

    starting_position = config['starting_position']
    explored_value = config['explored_value']
    unexplored_value = config['unexplored_value']
    state = config['state']
    steps = config['steps']
    s_range = config['surveillance_range']
    WPD = config['way_point_dropout']
    output_dir = config['output_dir']
    pgm_filename = config['pgm_filename']
    yaml_filename = config['yaml_filename']

    grid_map = map_generator.load_pgm(pgm_filename)
    yaml_data = map_generator.load_yaml(yaml_filename)
    
    print(f'total free space: {int(map_generator.estimate_area(grid_map,yaml_data,unexplored_value))} sq.mtrs')

    #map_generator.plot_rgb_map(grid_map)

    explore = Exploration(grid_map, s_range, unexplored_value, state, yaml_data)

    goals = explore.set_goals(starting_position, explored_value, unexplored_value, steps,WPD)
    #map_generator.plot_iterations(goals)
    wp = []
    ori = []
    for item in goals:
        iteration = item['iteration']
        goal = item['goals']['goal']
        orient = item['goals']['orientation']
        graph = item['graph']
        frontiers = item['frontiers']  
        title = f'Iteration {iteration} - Goal: {goal}'
        wp.append(goal)
        ori.append(orient)
        
        
        #map_generator.plot_rgb_map(graph, goal, title, frontiers =True) #save_fig = iteration )
    file_path = os.path.join(output_dir,'wp_ori_data.txt')

    # Writing to the file
    with open(file_path, 'w') as f:
        f.write(f"wp = {wp}\n")
        f.write(f"ori = {ori}\n")
    t_c_rot = sum(ori)
    resolution = yaml_data['resolution']
    optimizer = WayPointOptimizer(wp, 0.3, 5)
    best_wp =optimizer.run()
    metrics = calculate_and_plot_path(grid_map,wp,best_wp,resolution,t_c_rot,output_dir)
    print(metrics)
    
if __name__ == '__main__':
    main()
