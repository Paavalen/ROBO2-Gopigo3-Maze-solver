#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

#def callback(msg, sensor_number):

#    rospy.loginfo("Sensor %d: Distance=%.2f meters", sensor_number, msg.range)

#def main():
   # rospy.init_node('distance_subscriber')

   # rospy.Subscriber('/distance_sensor/distance1', Range, callback)
   # rospy.Subscriber('~distance2', Range, callback, callback_args=2)
   # rospy.Subscriber('~distance3', Range, callback, callback_args=3)

    #rospy.spin()

class DistanceSubscriber:
    def __init__(self):
        rospy.init_node('distance_subscriber')
        rospy.Subscriber("/distance_sensor/distance1", Range, self.callback1)
        rospy.Subscriber("/distance_sensor/distance2", Range, self.callback2)
        rospy.Subscriber("/distance_sensor/distance3", Range, self.callback3)
        self.range_values = [None, None, None]

    def callback1(self, data):
        self.range_values[0] = data.range

    def callback2(self, data):
        self.range_values[1] = data.range

    def callback3(self, data):
        self.range_values[2] = data.range


    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            for i, value in enumerate(self.range_values):
                if value is not None:
                    rospy.loginfo("Range value %d: %.2f", i+1, value)
                else:
                    rospy.logwarn("No range value received for sensor %d yet.", i+1)
            rate.sleep()


if __name__ == '__main__':
    try:
        subscriber = DistanceSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
