import os
import cv2
import numpy as np 
import rospy
import tensorflow as tf

from cv_bridge import CvBridge

from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image

THRESHOLD_SCORE = 0.6
NUM_CLASSES = 4
class TLClassifier(object):

    def __init__(self, is_site):

        # Load Classifer - Depending on Location [Site/Simulator]
        if is_site == False:
            self.threshold_score = THRESHOLD_SCORE
            rospy.logwarn("STATUS: SIMULATOR")
            ssd_model = os.path.abspath(os.curdir)+"/light_classification/frozen_model/frozen_sim_inception/frozen_inference_graph.pb"

        else:
            self.threshold_score = THRESHOLD_SCORE
            rospy.logwarn("STATUS: SITE")
            ssd_model = os.path.abspath(os.curdir)+"/light_classification/frozen_model/frozen_real_c2_1/frozen_inference_graph.pb"
        
        self.sess = None
        self.bridge = CvBridge()
        self.detection_graph = tf.Graph()

        # Tensorflow
        with self.detection_graph.as_default():
            classifier_graph = tf.GraphDef()

            with tf.gfile.GFile(ssd_model, 'rb') as fid:
                serialized_graph = fid.read()
                classifier_graph.ParseFromString(serialized_graph)
                tf.import_graph_def(classifier_graph, name='')

            self.sess = tf.Session(graph=self.detection_graph)

        # Classifier Elements
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        rospy.loginfo("Traffic Light Classifier is Loaded!")


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Predict Color of Traffic Light
        state =  TrafficLight.UNKNOWN
        # image = self.load_image_into_numpy_array(image)
        image_np_expanded = np.expand_dims(image, axis=0)

        #Predict
        with self.detection_graph.as_default():
            boxes, scores_array, classes_array, num_detections = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

        #Here is the pbtxt format:
        
        ''' 
        item {
            id: 1
            name: 'Green'
        }

        item {
            id: 2
            name: 'Red'
        }

        item {
            id: 3
            name: 'Yellow'
        }

        item {
            id: 4
            name: 'off'
        }
        '''

        #Process Scores
        scores = np.array([s for s in scores_array[0] if s > self.threshold_score])

        if len(scores) >= 1:
            classes = classes_array[0,0:len(scores)].astype('int32')

            # If any RED, send RED:
            if (classes==2).any():
                state = TrafficLight.RED
            
            else:
                counts = np.bincount(classes)
                most_class = np.argmax(counts)
                if most_class == 1:
                    state = TrafficLight.GREEN
                elif most_class == 3:
                    state = TrafficLight.YELLOW
                
        return state