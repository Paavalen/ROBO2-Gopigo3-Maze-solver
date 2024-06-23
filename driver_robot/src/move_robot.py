#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

#def callback(msg, sensor_number):
 #   rospy.loginfo("Sensor %d: Distance=%.2f meters", sensor_number, msg.range)

#def main():
 #   rospy.init_node('distance_subscriber')

  #  rospy.Subscriber('~distance1', Range, callback, callback_args=1)
   # rospy.Subscriber('~distance2', Range, callback, callback_args=2)
    #rospy.Subscriber('~distance3', Range, callback, callback_args=3)

   # rospy.spin()


class RobotController:
    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=True)
        self.sensor_values = {}
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/distance_sensor/distance1', Range, self.callback1)
        rospy.Subscriber('/distance_sensor/distance2', Range, self.callback2)
        rospy.Subscriber('/distance_sensor/distance3', Range, self.callback3)
        self.range_values = [None, None, None]
        self.ang_speed = 0
        self.target_angle = 40

    def callback1(self, data):
        self.range_values[0] = data.range

    def callback2(self, data):
        self.range_values[1] = data.range

    def callback3(self, data):
        self.range_values[2] = data.range


    def move_robot(self, linear_speed, angular_speed):

        def turn_left():
            print("left")
            while (rospy.Time.now() - start_time).to_sec() < 2:
                  twist_msg = Twist()
                  twist_msg.angular.z = 1
        
        def turn_right():
            print("right")
            while (rospy.Time.now() - start_time).to_sec() < 2:
                  twist_msg = Twist()
                  twist_msg.angular.z = -1


        def stop():
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0

        rate = rospy.Rate(8)  # 1 Hz

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed

        #while not rospy.is_shutdown():
            # Check if the front sensor value is less than 10
            #if self.range_values[0]  is not None:
               #value1 right 
               #if self.range_values[1] < 30:
                  #print('cond1')
        start_time = rospy.Time.now()
        turn_left()
               
               #if self.range_values[1] < 30 and self.range_values[0] < 30:
                  #print('cond2')
                  #start_time = rospy.Time.now()
                  #turn_left()
                  
               #if self.range_values[1] > 20 and self.range_values[1] < 80:
                  #print('forward')
                  #twist_msg = Twist()
                  #twist_msg.linear.x = linear_speed
                  #twist_msg.angular.z = angular_speed
               #elif self.range_values[1] > 80:
                  #print('right')
                  #start_time = rospy.Time.now()
                  #turn_right()

        self.cmd_vel_pub.publish(twist_msg)
        rate.sleep()
 

if __name__ == '__main__':
    try:
        controller = RobotController()
        # Move forward with linear speed of 0.1 m/s and no angular speed
        controller.move_robot(0, 0)
    except rospy.ROSInterruptException:
        pass


            # Check if the front sensor value is less than 10
           #  if self.sensor_value is not None and self.sensor_value < 10:
                # Stop the robot
             #   twist_msg.linear.x = 0
             #   twist_msg.angular.z = 0
