#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from FireBotMAP import Map_generator
import yaml
from FireBotScout_Multi_Processing import Exploration
from FireBot_path_optimizer import WayPointOptimizer

# Load map and YAML configuration
map_generator = Map_generator()
starting_position = [(200, 200)]
explored_value = 255 
unexplored_value = 254 
state = explored_value 
steps = 4
surveillance_range = 40  
pgm_filename = "/path/to/third_map.pgm"
yaml_filename = '/path/to/third_map.yaml'

grid_map = map_generator.load_pgm(pgm_filename)
yaml_data = map_generator.load_yaml(yaml_filename)

class RobotController:
    def __init__(self):
        self.fire_detected = False
        self.battery_level = 100
        self.state = "idle"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

    def move_to_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.orientation.w = y
        self.client.send_goal(goal)
        print(f"Moving to goal: ({x},{y})")
        self.state = "moving_to_goal"

    def stop_robot(self):
        self.client.cancel_all_goals()
        print("Stopping the robot by canceling the goal.")
        self.state = "idle"

    def fire_detection_callback(self, msg):
        self.fire_detected = (msg.data == 1)
        if self.fire_detected:
            self.handle_fire_detection()

    def charging_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level < 10:
            self.handle_low_battery()

    def handle_fire_detection(self):
        print("Fire detected! Activating fire extinguishing mode")
        if self.state == "idle" and self.battery_level >= 10:
            self.execute_fire_extinguishing()

    def handle_low_battery(self):
        print("Battery level below 10! Activating charging mode")
        if self.state != "charging":
            self.execute_charging()

    def execute_fire_extinguishing(self):
        self.stop_robot()
        

    def execute_charging(self):
        self.stop_robot()
       

    def switch_to_surveillance_mode(self):
        if self.state == "idle":
            print("Robot is idle, switching to exploration mode")
            self.explore()

    def explore(self):
        explore = Exploration(grid_map, surveillance_range, unexplored_value, state, yaml_data)
        surveillance_info = explore.set_goals(starting_position, explored_value, unexplored_value, steps)
        goals = [item['goal'] for item in surveillance_info]
        grasp_solver = WayPointOptimizer(goals)
        best_path = grasp_solver.run()

        for goal in best_path:
            if self.fire_detected or self.battery_level < 10:
                break
            self.move_to_goal(goal['x'], goal['y'])
            self.client.wait_for_result()  

        if not self.fire_detected and self.battery_level >= 10:
            self.switch_to_surveillance_mode()

def main():
    rospy.init_node('robot_controller', anonymous=True)
    controller = RobotController()

    rospy.Subscriber('/fire', Int32, controller.fire_detection_callback)
    rospy.Subscriber('/battery', Int32, controller.charging_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
