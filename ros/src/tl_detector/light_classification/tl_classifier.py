import os
import cv2
import numpy as np 
import rospy
import tensorflow as tf

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from cv_bridge import CvBridge

from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image

THRESHOLD_SCORE = 0.5
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

        #Load Label Map
        labelmap_dir = ssd_model = os.path.abspath(os.curdir)+"/light_classification/frozen_model/label_map.pbtxt"
        label_map = label_map_util.load_labelmap(labelmap_dir)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.stream_detection = rospy.Publisher('/tl_classifier_stream', Image, queue_size=1)

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

            # Create visualization
            vis_util.visualize_boxes_and_labels_on_image_array(
                image,
                np.squeeze(boxes),
                np.squeeze(classes_array).astype(np.int32),
                np.squeeze(scores_array),
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=8)

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

        #Publish Object Detection Visualization
        pub_img = self.bridge.cv2_to_imgmsg(image)
        self.stream_detection.publish(pub_img)

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
        
    # def load_image_into_numpy_array(self, image):

    #     (im_width, im_height) = image.size
    #     return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)