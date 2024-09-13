import os



# Configuration parameters for FireBot tool



config = {

    "starting_position": [(356, 33)],

    "explored_value": 150, # update cells in fov with explored value

    "unexplored_value": 254, # free space

    "state": 150,  # Should match explored_value

    "steps": 25, #number of iterations

    "surveillance_range": 100,  # Surveillance range (based on sensor range)

    "way_point_dropout": 0,  # Waypoint dropout

    "output_dir": "", # give path to save results

    "pgm_filename": "/map.pgm", #path where occupancy map is saved

    "yaml_filename": "/map.yaml", #path for yaml data

    "optimizer_params": {

        "wp_threshold": 0.3,

        "num_iterations": 5

    } # grasp threshold and num_of iterations

}

if not os.path.exists(config["output_dir"]):

    os.makedirs(config["output_dir"])
