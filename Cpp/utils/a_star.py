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

goals_f = [
    (350, 67),
    (324, 84),
    (311, 112),
    (321, 141),
    (316, 171),
    (296, 195),
    (268, 209),
    (244, 188),
    (262, 163),
    (245, 230),
    (214, 235),
    (196, 260),
    (170, 270),
    (253, 133),
    (253, 102),
    (222, 276),
    (234, 304),
    (235, 334),
    (261, 349),
    (290, 357),
    (300, 329),
    (275, 376),
    (327, 318),
    (318, 368),
    (317, 303),
    (248, 260),
    (326, 389),
    (331, 144),
    (183, 227),
    (262, 404),
    (231, 405),
    (178, 285),
    (331, 275),
    (332, 244),
    (245, 307),
    (293, 203),
    (210, 353),
    (229, 160),
    (235, 95),
    (290, 402),
    (221, 332),
    (262, 354),
    (223, 281),
    (304, 108),
    (333, 177),
    (328, 89),
    (306, 168),
    (317, 322),
    (233, 415),
    (304, 397),
    (153, 304),
    (207, 425),
    (193, 260),
    (222, 270),
    (210, 224),
    (159, 334),
    (213, 72),
    (154, 215),
    (270, 76),
    (222, 179),
    (250, 134),
    (200, 405),
    (262, 224),
    (318, 213),
    (276, 308),
    (276, 319),
    (225, 379),
    (354, 97),
    (333, 296),
    (318, 68),
    (345, 353),
    (254, 71),
    (200, 306),
    (257, 183),
    (257, 101),
    (144, 306),
    (278, 258),
    (294, 202),
    (205, 350),
    (299, 149),
    (147, 216),
    (338, 242),
    (328, 299),
    (159, 337),
    (182, 233),
    (202, 175),
    (317, 418),
    (192, 75),
    (250, 133),
    (254, 102),
    (332, 244),
    (267, 74),
    (255, 422),
    (226, 388),
    (305, 170),
    (220, 335),
    (255, 281),
    (328, 143),
    (348, 353),
    (190, 337)
]

goals_s = [
    (350, 67),
    (315, 102),
    (317, 152),
    (273, 177),
    (234, 210),
    (208, 254),
    (231, 299),
    (270, 331),
    (300, 371),
    (308, 297),
    (253, 129),
    (274, 379),
    (193, 258),
    (331, 251),
    (233, 408),
    (310, 373),
    (242, 265),
    (295, 108),
    (259, 215),
    (183, 228),
    (153, 290),
    (229, 349),
    (219, 98),
    (327, 154),
    (222, 185),
    (269, 164),
    (225, 431),
    (271, 337),
    (162, 330),
    (274, 313),
    (145, 215),
    (207, 68),
    (310, 152),
    (343, 247),
    (320, 335),
    (311, 102),
    (278, 77),
    (198, 384),
    (185, 228),
    (287, 420)
]
goals_ipa =[
    (321.879, 208.299), (307.373, 214.854), (301.818, 216.317), (293.256, 216.998), 
    (287.683, 211.818), (292.616, 199.342), (298.182, 197.683), (312.784, 222.657), 
    (321.889, 226.506), (315.076, 233.194), (321.971, 246.591), (326.386, 259.857), 
    (326.134, 272.972), (322.15, 293.976), (313.579, 309.949), (302.111, 312.506), 
    (308.619, 320.711), (320.858, 317.445), (339.984, 310.186), (350.62, 310.273), 
    (359.858, 310.555), (379.76, 313.934), (389.744, 310.998), (396.889, 306.494), 
    (390.113, 292.033), (378.143, 281.614), (369.066, 278.971), (354.142, 270.445), 
    (344.127, 252.348), (331.156, 240.423), (325.372, 235.14), (297.384, 235.342), 
    (282.267, 229.392), (274.038, 223.722), (268.614, 212.143), (266.124, 205.827), 
    (262.058, 187.046), (261.614, 174.143), (262.445, 168.142), (261.998, 160.256), 
    (255.349, 154.488), (247.122, 156.289), (236.066, 175.029), (230.818, 180.317), 
    (218.342, 174.384), (216.683, 168.818), (215.745, 158.966), (221.858, 153.555), 
    (233.658, 159.616), (233.404, 178.715), (235.86, 175.372), (244.919, 178.907), 
    (259.971, 166.409), (274.821, 160.311), (254.009, 156.081), (248.256, 156.002), 
    (258.151, 187.944), (268.935, 194.255), (250.249, 197.991), (244.066, 199.029), 
    (233.614, 222.857), (243.002, 232.744), (259.723, 237.629), (269.179, 240.896), 
    (293.656, 253.868), (305.658, 256.616), (307.889, 262.506), (302.094, 274.849), 
    (277.249, 269.998), (281.642, 266.501), (287.601, 257.167), (299.743, 279.621), 
    (301.889, 285.506), (297.384, 297.658), (289.195, 299.648), (285.138, 310.55), 
    (275.094, 318.849), (273.384, 318.658), (260.335, 319.313), (256.818, 322.317), 
    (246.055, 321.023), (228.255, 317.209), (225.315, 304.128), (225.342, 302.384), 
    (225.151, 290.094), (225.342, 288.384), (224.959, 277.05), (224.555, 271.142), 
    (239.745, 258.034), (245.858, 256.555), (257.658, 272.616), (257.744, 273.002), 
    (258.111, 283.494), (268.61, 270.472), (273.963, 269.903), (284.701, 264.663), 
    (273.04, 270.28), (287.018, 297.763), (327.625, 283.141), (338.878, 281.289), 
    (362.768, 288.883), (372.766, 293.428), (387.286, 299.651), (367.136, 297.649), 
    (348.152, 298.568), (338.256, 300.998), (335.124, 294.827), (335.381, 279.302), 
    (349.029, 234.066), (346.794, 224.269), (355.486, 222.617), (356.555, 217.142), 
    (367.909, 205.137), (369.616, 205.342), (382.706, 205.318), (388.858, 203.555), 
    (402.646, 237.616), (404.889, 243.506), (398.384, 255.658), (394.818, 259.317), 
    (381.225, 257.161), (375.818, 257.317), (363.133, 253.269), (363.25, 250), 
    (359.371, 237.395), (358.046, 228.005), (368.018, 205.237), (383.617, 205.486), 
    (359.157, 210.941), (357.855, 221.6), (374.62, 231.128), (378.651, 231.512), 
    (382.62, 228.128), (400.969, 217.608), (409.94, 229.23), (405.002, 238.744), 
    (415.386, 219.143), (423.998, 209.256), (413.046, 204.059), (402.133, 204.313), 
    (375.691, 226.813), (386.388, 229.891), (386.38, 234.128), (379.249, 228.304), 
    (378.302, 231.801), (361.002, 242.256), (374.658, 231.616), (375.982, 234.237), 
    (363.32, 244.432), (371.041, 256.044), (377.255, 234.065), (384.018, 234.237), 
    (402.763, 246.982), (402.14, 234.372), (384.149, 231.747), (401.827, 233.47), 
    (390.018, 255.763), (404.924, 244.461), (392.004, 255.572), (396.386, 271.857), 
    (399.265, 282.889), (402.386, 313.857), (393.028, 324.863), (378.214, 332.488), 
    (370.45, 342.242), (366.504, 333.102), (354.235, 324.465), (343.907, 322.397), 
    (336.745, 321.065), (334.998, 328.744), (323.238, 333.543), (320.143, 336.386), 
    (310.818, 337.317), (244.225, 332.161), (231.265, 329.397), (222.256, 319.002), 
    (218.539, 327.626), (221.557, 336.973), (232.651, 343.714), (236.237, 341.288), 
    (226.249, 332.27), (218.123, 305.031), (209.608, 286.031), (204.614, 281.143), 
    (184.729, 260.215), (178.45, 247.242), (176.174, 236.267), (167.235, 246.535), 
    (170.794, 264.702), (175.386, 269.857), (180.889, 275.506), (146.384, 287.658), 
    (93.871, 314.68), (96.158, 320.012), (106.028, 326.621), (120.316, 339.372), 
    (133.006, 356.915), (145.174, 369.107), (168.015, 375.639), (189.757, 368.823), 
    (200.636, 357.748), (213.209, 346.945), (223.663, 338.194), (232.248, 336.377), 
    (238.839, 343.42), (248.179, 342.842), (249.13, 340.504), (246.055, 329.834), 
    (251.801, 319.653), (258.34, 318.609), (259.644, 314.929), (255.821, 307.934), 
    (259.451, 297.183), (248.085, 296.126), (235.875, 291.762), (232.202, 274.125), 
    (228.292, 261.553), (220.654, 251.123), (213.482, 234.545), (210.155, 224.2), 
    (207.124, 211.593), (207.565, 204.621), (207.948, 191.496), (204.242, 179.468), 
    (189.272, 167.343), (179.487, 159.636), (164.58, 144.3), (161.743, 138.923), 
    (159.339, 136.178), (146.38, 131.845), (139.835, 130.748), (131.989, 131.644), 
    (124.221, 132.038), (121.21, 127.744), (117.786, 119.897), (115.601, 114.621), 
    (113.369, 104.426), (109.076, 99.62), (102.223, 86.982), (98.782, 75.391), 
    (94.128, 65.036), (83.017, 60.216), (80.81, 56.73), (75.594, 51.873), 
    (70.956, 49.657), (62.438, 41.095), (59.451, 39.763), (55.503, 32.155), 
    (53.098, 29.707), (50.838, 21.21), (53.939, 19.162), (54.684, 18.92), 
    (63.764, 26.565), (67.306, 32.428), (73.052, 37.856), (80.444, 41.24), 
    (88.606, 41.168), (93.576, 45.079), (100.63, 50.79), (104.511, 56.844), 
    (111.896, 62.297), (115.457, 69.066), (122.126, 71.015), (124.262, 79.168), 
    (120.548, 82.177), (118.49, 85.543), (113.553, 86.09), (112.259, 86.701), 
    (110.056, 85.339), (105.059, 82.219), (97.305, 76.565), (90.382, 71.25), 
    (85.78, 66.307), (79.018, 56.268), (74.862, 52.601), (68.868, 48.216), 
    (64.043, 43.407), (59.458, 40.743), (56.513, 37.836), (50.042, 29.021)
]


#goals = [(int(x), int(y)) for x, y in goals_ipa]
#optimized_goals = WayPointOptimizer(goals, 0.3, 5)
#best_path = optimized_goals.run()

#print(best_path)

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
    
map_data = load_pgm("/home/skachavarapu/home/Research_Track/house_ipa_results/map/map.pgm")
metadata = load_yaml("/home/skachavarapu/home/Research_Track/house_ipa_results/map/map.yaml")
#map_data = rotate_array_90_degrees(map_data)
#map_data = flip_array_vertically(map_data)
print(np.unique(map_data))
resolution = metadata['resolution']
origin = metadata['origin'][:2]  # Only need x and y

# Transform goal coordinates (these are placeholders, replace with actual transformation based on map)
goals_transformed = [
    # Placeholder transformation; replace with the actual transformation logic based on map's resolution and origin
]
goals = goals_s
goals1 = [(350, 67), (315, 102), (311, 102), (295, 108), (310, 152), (317, 152), (327, 154), (343, 247), (331, 251), (308, 297), (320, 335), (310, 373), (300, 371), (287, 420), (225, 431), (233, 408), (198, 384), (229, 349), (274, 379), (271, 337), (270, 331), (274, 313), (231, 299), (162, 330), (153, 290), (145, 215), (183, 228), (185, 228), (193, 258), (208, 254), (242, 265), (259, 215), (234, 210), (222, 185), (273, 177), (269, 164), (253, 129), (219, 98), (207, 68), (278, 77), (350, 67)]

before_wpo = True
after_wpo = True
#goals1 = best_path
total_path_length = 0
total_path_length1 = 0
fig, ax = plt.subplots(figsize=(10, 10))
b_map = convert_pgm_to_binary_custom(map_data)
cummulative_rotation = 574
area_map = load_pgm('area.pgm')
ax.imshow(b_map, cmap='gray')
grid1 = np.copy(map_data)
grid2= np.copy(map_data)

if before_wpo:
    for i in range(len(goals) - 1):
        path = a_star(grid1, goals[i], goals[i+1])
        if path:
            
            total_path_length += len(path) * resolution
            
            y, x = zip(*path)
            ax.plot(x, y, color='dimgray', alpha = 0.5)  
if after_wpo:
    for i in range(len(goals1) - 1):
        path1 = a_star(grid2, goals1[i], goals1[i+1])
        if path1:
            
            total_path_length1 += len(path1) * resolution
            
            y1, x1 = zip(*path1)
            ax.plot(x1, y1, color='r')


# Plotting goals
for goal in goals:
    ax.plot(goal[1], goal[0], 'b*', markersize=3)
 
t_t = revisit_time_(total_path_length1, cummulative_rotation, linear_speed = 0.3, rotational_speed= 0.52)
print(f'path_length {total_path_length1},cummulative rotations {cummulative_rotation}, revisit_time {t_t}')
#metrics = ['path_length',total_path_length1,'cummulative rotations',cummulative_rotation, 'revisit_time' ,t_t]
#csv_file = 'latest_results_3/ipa.csv'
#with open(csv_file,'w',newline = '') as file:
 #   writer = csv.writer(file)
  #  writer.writerow(metrics)
plt.legend()
plt.savefig('wp_candidates/Fare_cpp_before_and_after_wpo.png', dpi=200) 
plt.show()




