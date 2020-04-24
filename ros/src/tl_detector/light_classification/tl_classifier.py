import os
import cv2
import numpy as np 
import rospy
import tensorflow as tf

from styx_msgs.msg import TrafficLight


class TLClassifier(object):

    def __init__(self, model_path):
        # Initialize TLClassifier Class

        self.current_light = TrafficLight.UNKNOWN


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
