#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLight
from geometry_msgs.msg import TwistStamped, PoseStamped
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd

HZ_SETTING = 50

class DashboardDisplay(object):

    def __init__(self):

        rospy.init_node('dashboard_display')

        self.init_members()
        self.init_connections()
        self.is_init = False

        self.loop()
    
    def loop(self):

        while not rospy.is_shutdown():

            if(self.is_init):

                print("==============================[  DASHBOARD  ]==============================\n")
                rospy.loginfo("Steering Angle (Rad):                {:.5f}".format(self.steering))
                rospy.loginfo("Throttle Command (% of Max):         {:.5f}".format(self.throttle))
                rospy.loginfo("Braking Force (Nm):                  {:.5f}".format(self.brake))
                rospy.loginfo("Traffic Light Color:                 {0}".format(self.tl_status))
                rospy.loginfo("Gloabl X-Positon:                    {0}".format(self.pose.pose.position.x))
                rospy.loginfo("Gloabl Y-Positon:                    {0}".format(self.pose.pose.position.y))
                rospy.loginfo("Linear Velocity (m/s):               {:.5f}\n\n".format(self.velocity))
                print("==============================[  DASHBOARD  ]==============================\n")

            
            rospy.sleep(1.)

    def init_connections(self):

        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steer_cb)
        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.throttle_cb)
        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.brake_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/tl_state', TrafficLight, self.tl_cb)

    def init_members(self):
        #Initialize Dashboard Info

        self.steering = None
        self.throttle = None
        self.brake = None
        self.traffic_idx = ('RED', 'YELLOW', 'GREEN', None, 'N/A')
        self.tl_status = self.traffic_idx[4]
        self.pose = PoseStamped()
        self.velocity = None

    def steer_cb(self, msg):
        #Call back function for steering data

        self.steering = msg.steering_wheel_angle_cmd

    def throttle_cb(self, msg):
        #Call back function for throttle data

        self.throttle = msg.pedal_cmd

    def brake_cb(self, msg):
        #Call back function for brake data

        self.brake = msg.pedal_cmd

    def pose_cb(self, msg):
        #Call back function for position data

        self.pose = msg

    def velocity_cb(self, msg):
        #Call back function for velocity data

        self.velocity = msg.twist.linear.x

    def tl_cb(self, msg):
        #Call back function for traffic light status

        self.tl_status = self.traffic_idx[int(msg.state)]
        self.is_init = True

if __name__ == '__main__':
    DashboardDisplay()
