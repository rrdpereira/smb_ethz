#!/usr/bin/env python3
import rospy
import std_srvs.srv
from nav_msgs.msg import Odometry
from sensor_fusion_comm.srv import InitScale

# Note: this is currently only implemented for the initialize_msf_scale service type and an nav_msgs::Odometry input message

class MSFAutoInit(object):
    def __init__(self):
        # ROS Params
        self.threshold_msg_no_for_init = rospy.get_param("/msf_autoinit/threshold_msg_no_for_init", 5)
        self.input_topic_name = rospy.get_param("/msf_autoinit/input_topic_name", "/loam/odometry")
        self.init_service_name = rospy.get_param("/msf_autoinit/init_service_name", \
            "/msf_loam_body_imu/msf_loam_body_imu/pose_sensor/initialize_msf_scale")
        self.init_scale = rospy.get_param("/msf_autoinit/init_scale", 1.0)
        # ROS Subscribers
        rospy.Subscriber(self.input_topic_name, Odometry, self.input_message_callback, queue_size=1)
        # Class Vars
        self.input_message_counter = 0
        self.got_the_service = False
    def initialize_msf(self):
        # Initializing MSF with IMU and pose sensor
        srv_success = True
        print("Trying to initialize MSF upon receiving input odometry messages")
        try:
            init_msf_scale = rospy.ServiceProxy(self.init_service_name, InitScale)
            resp = init_msf_scale(self.init_scale)
        except rospy.ServiceException as e:
            srv_success = False
            print("Service call for initialization of MSF with pose sensor and IMU failed: %s"%e)
        if(srv_success):
            # Shutdown of the init-node:
            print("Successfully auto-initialized MSF, shutting down the autoinit node")
            rospy.signal_shutdown('MSF initializing service was called')
    def input_message_callback(self, input_msg):
        if self.got_the_service:
            self.increment_counter()
        else:
            # Waiting for the service to appear first for robustness
            print("Waiting for initialization service")
            rospy.wait_for_service(self.init_service_name)
            self.got_the_service = True
            return
    def increment_counter(self):
        # Initializing MSF upon arrival of minimum number of input odometry messages
        self.input_message_counter += 1
        print("Got: " + str(self.input_message_counter) + " input odometry messages")
        if (self.input_message_counter >= self.threshold_msg_no_for_init and self.got_the_service):
            self.initialize_msf()
if __name__ == '__main__':
    rospy.init_node('msf_autoinit', disable_signals=True)
    MSFAutoInit()
    while not rospy.is_shutdown():
        rospy.spin()