#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree

import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    
    def __init__(self):

        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=5)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string, Loader=yaml.FullLoader)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.traffic_light_color_pub = rospy.Publisher('/tl_state', TrafficLight, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config["is_site"])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints] 
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        tf_msg = TrafficLight()

        # Call Processor to Identify the Light Waypoint and its state
        light_wp, tf_msg.state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        # Initialize State
        if self.state != tf_msg.state:
            self.state_count = 0
            self.state = tf_msg.state
        
        # Ensure the is some level in confidence of the state before changing the state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            # If the state of the traffic light is RED then stop, else continue driving
            light_wp = light_wp if tf_msg.state == TrafficLight.RED else -1
            
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))

        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1
        self.traffic_light_color_pub.publish(tf_msg)

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        if self.config["is_site"] == False:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
        else:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        try:
            #Get classification
            traffic_light_class = self.light_classifier.get_classification(cv_image)

        except AttributeError:
            return False

        return traffic_light_class

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if(self.pose):

            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            # Find the closest visible traffic light (if one exists) by iterating through all lights [only 8]
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                
                #Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0],line[1]) # (x, y)
                diff_in_idx = (temp_wp_idx - car_wp_idx) % len(self.waypoints.waypoints)

                #Check if diff_in_idx is ahead or behind
                if diff_in_idx >= 0 and diff_in_idx < diff:
                    diff = diff_in_idx
                    closest_light = light
                    line_wp_idx = temp_wp_idx
            
        #Check the state of the light - COLOR
        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
