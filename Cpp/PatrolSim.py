#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import yaml
import os
from config import config
from FireBotMAP import Map_generator
map_generator = Map_generator()
output_dir = config['output_dir']
pgm_filename = config['pgm_filename']
yaml_filename = config['yaml_filename']
map_data = map_generator.load_yaml(yaml_filename)
waypoints_path = os.path.join(output_dir,'wp_ori_data.txt')

def send_goal(x, y, theta):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = theta  
    goal.target_pose.pose.orientation.w = 1.0 

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Failed to reach the goal")
    else:
        rospy.loginfo("Reached the goal")


def grid_to_world_coords(wp, map_data):
    resolution = map_data['resolution']
    origin = map_data['origin']
    
    world_coords = []
    for cell in wp:
        x = cell[0] * resolution + origin[0]
        y = cell[1] * resolution + origin[1]
        world_coords.append((x, y))
    return world_coords


def patrol():
    rospy.init_node('patrol_waypoints')
    with open(waypoints_path, 'r') as f:
        lines = f.readlines()
        wp = eval(lines[0].split('=')[1].strip())  # Waypoints from occupancy grid
        ori = eval(lines[1].split('=')[1].strip())  # Orientations in radians

    # Convert waypoints from grid cells to world coordinates
    world_waypoints = grid_to_world_coords(wp, map_data)
    try:
        for i, (x, y) in enumerate(world_waypoints):
            rospy.loginfo(f"Sending goal {i+1}/{len(world_waypoints)}: ({x}, {y}, {ori[i]})")
            send_goal(x, y, ori[i])

        rospy.loginfo("Patrolling is finished")

    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping patrolling due to interruption")

if __name__ == '__main__':
    try:
        patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")
