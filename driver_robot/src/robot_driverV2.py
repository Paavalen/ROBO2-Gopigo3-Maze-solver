#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import numpy as np

class RobotController:
    def __init__(self, maze_size, goal_position):
        rospy.init_node('move_robot_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
       
        rospy.Subscriber('/distance_sensor/distance1', Range, self.callback1)  # Front sensor
        rospy.Subscriber('/distance_sensor/distance2', Range, self.callback2)  # Left sensor
        rospy.Subscriber('/distance_sensor/distance3', Range, self.callback3)  # Right sensor
       
        self.range_values = [None, None, None]
        self.linear_speed = 0.1
        self.angular_speed = 0.5
       
        self.maze_size = maze_size
        self.goal_position = goal_position
        self.maze = np.full(maze_size, np.inf)  # Initialize maze distances with infinity
        self.maze[goal_position] = 0  # Set goal position distance to 0
       
        self.rate = rospy.Rate(10)  # 10 Hz

    def callback1(self, data):
        self.range_values[0] = data.range

    def callback2(self, data):
        self.range_values[1] = data.range

    def callback3(self, data):
        self.range_values[2] = data.range

    def move_robot(self, linear_speed, angular_speed):
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist_msg)

    def turn_left(self):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 0.7:  # Adjusted turn duration
            self.move_robot(0, self.angular_speed)
            self.rate.sleep()

    def turn_right(self):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 0.7:  # Adjusted turn duration
            self.move_robot(0, -self.angular_speed)
            self.rate.sleep()

    def stop(self):
        self.move_robot(0, 0)

    def update_maze(self, current_position):
        # Update the distances in the maze using flood fill
        queue = [self.goal_position]
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
       
        while queue:
            x, y = queue.pop(0)
            current_distance = self.maze[x, y]
           
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.maze_size[0] and 0 <= ny < self.maze_size[1]:
                    if self.maze[nx, ny] > current_distance + 1:
                        self.maze[nx, ny] = current_distance + 1
                        queue.append((nx, ny))

    def get_next_move(self, current_position):
        x, y = current_position
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
        best_direction = None
        min_distance = np.inf
       
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.maze_size[0] and 0 <= ny < self.maze_size[1]:
                if self.maze[nx, ny] < min_distance:
                    min_distance = self.maze[nx, ny]
                    best_direction = (dx, dy)
       
        return best_direction

    def solve_maze(self):
        current_position = (0, 0)  # Starting position (example)
       
        while not rospy.is_shutdown():
            front_dist = self.range_values[0]
            left_dist = self.range_values[1]
            right_dist = self.range_values[2]

            if front_dist is not None and left_dist is not None and right_dist is not None:
                if front_dist < 30:  # Adjusted threshold for front sensor
                    self.stop()
                    rospy.loginfo("Obstacle ahead")
                    if right_dist > 30:  # Adjusted threshold for right sensor
                        rospy.loginfo("Turning right")
                        self.turn_right()
                    elif left_dist > 30:  # Adjusted threshold for left sensor
                        rospy.loginfo("Turning left")
                        self.turn_left()
                else:  # Path is clear
                    next_move = self.get_next_move(current_position)
                    if next_move is not None:
                        dx, dy = next_move
                        if dx == 1:
                            rospy.loginfo("Moving down")
                        elif dx == -1:
                            rospy.loginfo("Moving up")
                        elif dy == 1:
                            rospy.loginfo("Moving right")
                        elif dy == -1:
                            rospy.loginfo("Moving left")
                       
                        self.move_robot(self.linear_speed, 0)
                        current_position = (current_position[0] + dx, current_position[1] + dy)
                    else:
                        rospy.loginfo("No valid moves available")
            else:
                rospy.loginfo("Waiting for sensor data")

            self.update_maze(current_position)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        cell_size = 0.33  # 10 cm per cell
        physical_width = 1.5  # meters
        physical_height = 2.0  # meters
       
        maze_width = int(physical_width / cell_size)
        maze_height = int(physical_height / cell_size)
       
        maze_size = (maze_width, maze_height)
        start_position = (1, 1)
        goal_position = (maze_width - 1, maze_height - 1)  # Adjust as needed for your maze
       
        controller = RobotController(maze_size, goal_position)
        controller.solve_maze()
    except rospy.ROSInterruptException:
        pass
