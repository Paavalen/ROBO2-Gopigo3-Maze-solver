#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

# Define GPIO pins for each sensor
PIN_SENSOR_1_TRIGGER = 16
PIN_SENSOR_1_ECHO = 24

PIN_SENSOR_2_TRIGGER = 21
PIN_SENSOR_2_ECHO = 12

PIN_SENSOR_3_TRIGGER = 6
PIN_SENSOR_3_ECHO = 20

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    # Set up trigger pins as output and echo pins as input
    GPIO.setup(PIN_SENSOR_1_TRIGGER, GPIO.OUT)
    GPIO.setup(PIN_SENSOR_1_ECHO, GPIO.IN)
    
    GPIO.setup(PIN_SENSOR_2_TRIGGER, GPIO.OUT)
    GPIO.setup(PIN_SENSOR_2_ECHO, GPIO.IN)
    
    GPIO.setup(PIN_SENSOR_3_TRIGGER, GPIO.OUT)
    GPIO.setup(PIN_SENSOR_3_ECHO, GPIO.IN)

def read_distance(trigger_pin, echo_pin):
    # Send trigger pulse
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, GPIO.LOW)
    
    # Wait for echo to start
    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
    
    # Wait for echo to end
    end_time = time.time()
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()
    
    # Calculate distance
    duration = end_time - start_time
    distance = duration * 34300 / 2  # Speed of sound = 343 m/s (in air), divide by 2 because echo travels back and forth
    return distance

def main():
    rospy.init_node("distance_sensor")
    setup_gpio()

    pub_distance1 = rospy.Publisher("~distance1", Range, queue_size=10)
    pub_distance2 = rospy.Publisher("~distance2", Range, queue_size=10)
    pub_distance3 = rospy.Publisher("~distance3", Range, queue_size=10)

    rate = rospy.Rate(rospy.get_param('~hz', 1))
    while not rospy.is_shutdown():
        # Read distance from each sensor
        distance1 = read_distance(PIN_SENSOR_1_TRIGGER, PIN_SENSOR_1_ECHO)
        distance2 = read_distance(PIN_SENSOR_2_TRIGGER, PIN_SENSOR_2_ECHO)
        distance3 = read_distance(PIN_SENSOR_3_TRIGGER, PIN_SENSOR_3_ECHO)

        # Publish distances for each sensor
        pub_distance1.publish(range_message(distance1, "~distance1"))
        pub_distance2.publish(range_message(distance2, "~distance2"))
        pub_distance3.publish(range_message(distance3, "~distance3"))

        rate.sleep()

def range_message(distance, frame_id):
    msg = Range()
    msg.header.frame_id = frame_id
    msg.radiation_type = Range.INFRARED
    msg.min_range = 0.02
    msg.max_range = 3.0
    msg.range = distance
    msg.header.stamp = rospy.Time.now()
    return msg

if __name__ == '__main__':
    try:
        main()
    finally:
        GPIO.cleanup()  # Cleanup GPIO pins when exiting
