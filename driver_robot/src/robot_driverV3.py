#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class RobotController:
    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
       
        rospy.Subscriber('/distance_sensor/distance1', Range, self.callback1)  # Front sensor
        rospy.Subscriber('/distance_sensor/distance2', Range, self.callback2)  # Left sensor
        rospy.Subscriber('/distance_sensor/distance3', Range, self.callback3)  # Right sensor
       
        self.range_values = [None, None, None]
       
        self.linear_speed = 0.2
        self.angular_speed = 1
        self.proportional_gain = 0.01  # Proportional gain for wall following
        
        self.desired_distance = 10  # Desired distance from the left wall (meters)
       
        self.rate = rospy.Rate(8)  # 10 Hz
        
        self.backtracking = False  # Flag to indicate if robot is backtracking
        self.backtrack_count = 0   # Counter to limit backtrack attempts

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

        omega = self.angular_speed  # Angular velocity in radians/second
        angle_to_turn = math.pi / 2  # 90 degrees in radians
        
        turn_duration = angle_to_turn / omega
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < turn_duration - 0.2:
            self.move_robot(0, self.angular_speed)
            self.rate.sleep()
        
        self.stop()

    def turn_right(self):

        omega = self.angular_speed  # Angular velocity in radians/second
        angle_to_turn = math.pi / 2  # 90 degrees in radians
        
        turn_duration = angle_to_turn / omega
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < turn_duration - 0.2:
            self.move_robot(0, -self.angular_speed)
            self.rate.sleep()
        
        self.stop()

    def stop(self):
        self.move_robot(0, 0)

    def spin_to_find_wall(self):
        rospy.loginfo("Spinning to find left wall")
        while self.range_values[1] is None or self.range_values[1] > 26:
            self.move_robot(0, self.angular_speed)
            self.rate.sleep()
        rospy.loginfo("Found left wall, starting to follow")

    def solve_maze(self):
        rospy.loginfo("Starting maze-solving behavior")
        while not rospy.is_shutdown():
            if self.range_values[0] is not None and self.range_values[1] is not None and self.range_values[2] is not None:
                front_dist = self.range_values[0]
                left_dist = self.range_values[1]
                right_dist = self.range_values[2]
                
                if front_dist < 22:  # If obstacle in front, stop and turn right
                    rospy.loginfo("Obstacle ahead")
                    self.stop()
                    
                    if left_dist > 20 and left_dist > right_dist:
                        rospy.loginfo("turn left")
                        self.turn_right()
                    if right_dist > 20 and right_dist > left_dist:
                        rospy.loginfo("turn right")
                        self.turn_left()

                          
                    elif self.is_dead_end():  # If in a dead-end, backtrack
                        self.backtrack()
                        
                            
                else:  # Move forward and adjust based on left sensor
                    if left_dist < self.desired_distance:  # Too close to left wall, turn right
                        self.move_robot(self.linear_speed * 0.55, -self.angular_speed * self.proportional_gain)
                    elif left_dist > self.desired_distance:  # Too far from left wall, turn left
                        self.move_robot(self.linear_speed * 0.55, self.angular_speed * self.proportional_gain)
                    else:  # Move forward if at desired distance from left wall
                        self.move_robot(self.linear_speed, 0)
            
            self.rate.sleep()

        rospy.loginfo("Maze-solving behavior ended")
        
        # all 3 sensors: backtrack for 3 seconds, turn right
        # only front sensor: stop, turn right 90 degrees

    def is_dead_end(self):
        # Method to determine if the robot is truly stuck in a dead-end
        front_dist = self.range_values[0]
        left_dist = self.range_values[1]
        right_dist = self.range_values[2]
        
        # Check conditions for a dead-end
        if (front_dist < 26 and left_dist < 26) and (front_dist < 26 and right_dist < 26):
            return True
        
        return False
        
    
    def backtrack(self):
        if not self.backtracking:
            rospy.loginfo("Initiating backtracking")
            self.backtracking = True
        
        # Move backward slowly for a short distance
        backtrack_duration = 4 # Duration to move backward (seconds)
        backtrack_speed = -0.1  # Speed to move backward
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < backtrack_duration:
            self.move_robot(backtrack_speed, 0)
            self.rate.sleep()
        
        # Check left distance after moving backward
        left_dist = self.range_values[1]
        right_dist = self.range_values[2]
        
        self.stop()
        
        
        if left_dist > 10:
            self.turn_right()
        elif right_dist > 10:
            self.turn_left()
        
        elif self.is_dead_end(): # If no clear path found, continue to backtrack
            rospy.loginfo("No clear path found, continuing to backtrack")
            # Increase backtrack counter
            self.backtrack_count += 1
            
            # Reset backtracking flag to stop backtracking
            self.backtracking = False

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.solve_maze()
    except rospy.ROSInterruptException:
        pass

  

