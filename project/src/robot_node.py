#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys, time
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyRequest
import tf

imu_topic = "/imu"

class RobotNode:
    def __init__(self):
        self.imu_subscriber = rospy.Subscriber(imu_topic,Imu,self.imuCallback, queue_size=1)
        self.curr_pitch_pub = rospy.Publisher('/curr_pitch', Float32, queue_size=1)

    def imuCallback(self, data):
        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        
        rospy.loginfo("roll: %.4f pitch: %.4f yaw: %.4f", roll, pitch, yaw)
        self.curr_pitch_pub.publish(pitch)  
        if abs(pitch) > 0.4:
            rospy.loginfo( "Pitch is greater than 0.4. Sequence is completed.")

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('robot_node', anonymous=True)
    env = RobotNode()
    
    rate = rospy.Rate(1)
    try:
        rospy.spin()
        rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down ROS ")    
        sys.exit()

if __name__ == '__main__':
    main(sys.argv)
