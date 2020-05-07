#!/usr/bin/env python

import numpy as np
import math
from scipy.spatial import KDTree

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish.
MAX_DECEL = 0.5 # m/s^2 Max Deceleration
HZ_SETTING = 50 # Hz Publishing Rate
STOP_NUM_WP_BACK = 2 # How many waypoints must the car stop before light

class WaypointUpdater(object):
    
    def __init__(self):
        #Initialization
        
        rospy.init_node('waypoint_updater')

        # Member Variables
        self.base_lane = None
        self.obstacle_wp_idx = -1
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        
        # Initialize Subscribers and Publishers
        self.init_connections()
        #Rather than rospy.spin(), manage the publishing frequency manually.
        self.loop()

    
    def init_connections(self):
        #Initialize Subscribers and Publishers

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


    def pose_cb(self, msg):   
        #Call back function for position

        self.pose = msg


    def waypoints_cb(self, msg):
        #Call back function for waypoints

        self.base_lane = msg
        # Initialize self.waypoints_2d before subscriber in order to prevent callback before intialized
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] 
                                for waypoint in msg.waypoints] 
            self.waypoint_tree = KDTree(self.waypoints_2d) # For O(log(n)) search


    def traffic_cb(self, msg):
        #Call back function for traffic light index

        self.stopline_wp_idx = msg.data # Index of waypoint closest to traffic light 

    
    def obstacle_cb(self, msg):
        # Callback for obstacle index
        
        self.obstacle_wp_idx = msg.data # Index of waypoint closest to obstacle

    
    def loop(self):
        # Enforce a standard Publishing Rate
        
        rate = rospy.Rate(HZ_SETTING)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()


    def get_closest_waypoint_index(self):
        # Return the Index of the closest waypoint 

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Return 1 item which is closest t [x,y], query returns [item, index] so use [1]
        closest_index = self.waypoint_tree.query([x,y],1)[1]

        # Check if the waypoint is ahead or behind vehicle.
        closest_coord = self.waypoints_2d[closest_index]
        prev_coord = self.waypoints_2d[closest_index - 1]

        # Eq for hyperplane through closest_coord
        close_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        '''
        -> If the angle between A and B are greater than 90 degrees, the dot product will be negative (less than zero)
        -> pos_vect can either be between prev_vect and close_vect or in front of close_vect
        -> If val < 0, angle is less than 90 so close_vect is infront of pos_vect
        -> If val > 0, angle is greater than 90 so pos_vect is ahead of close_vect
        '''
        
        val = np.dot(close_vect - prev_vect, pos_vect - close_vect)
        
        # If the waypoint is behind, use next point
        if val > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        
        return closest_index
    

    def publish_waypoints(self):
        # Publish next set of waypoints

        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)


    def generate_lane(self):
        # Generate the next set of way points

        lane = Lane()
        closest_idx = self.get_closest_waypoint_index()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        # If there is no traffic light nearby or green light, continue path. 
        if self.stopline_wp_idx >= farthest_idx:
            lane.waypoints = base_waypoints

        # If yellow light
        elif self.stopline_wp_idx < 0:
            stop_idx = min(max(self.stopline_wp_idx - closest_idx - 2, 0), len(base_waypoints)-1)
            dist_to_yellow = self.distance(base_waypoints, 0, stop_idx)

            # Run yellow if feasible
            if dist_to_yellow < 2.5 * base_waypoints[0].twist.twist.linear.x:
                lane.waypoints = base_waypoints
            # Else stop
            else:
                lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        # Red Light, plan for deceleration
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        
        return lane


    def decelerate_waypoints(self, waypoints, closest_idx):
        # For each waypoint, slow downt the velocity

        decelerated_wp = []
        
        for i, wp in enumerate(waypoints):
            # Preserve Position
            new_wp = Waypoint()
            new_wp.pose = wp.pose

            # Stop Index, -2 to stop a little behind the stopline
            stop_idx = max(self.stopline_wp_idx - closest_idx - STOP_NUM_WP_BACK, 0)
            distance = self.distance(waypoints, i, stop_idx)
            
            # Increasing Deceleration per Iteration (Uniform Deceleration): v^2 = 2*a*x
            velocity = math.sqrt(2 * MAX_DECEL * distance)
            if velocity < 1.:
                velocity = 0.

            new_wp.twist.twist.linear. x = min(velocity, wp.twist.twist.linear.x)
            decelerated_wp.append(new_wp)
        
        return decelerated_wp


    def distance(self, waypoints, wp1, wp2):
        #Compute Total Distance between two waypoints using sum of segment lengths

        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
