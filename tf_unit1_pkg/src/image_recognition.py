#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tensorflow as tf
import classify_image
from geometry_msgs.msg import Point, PoseStamped, PointStamped
import numpy as np
import math
import rosbag
import sys



class RosTesnsorFlow():
    def __init__(self):
        classify_image.maybe_download_and_extract()
        self._session = tf.session()
        classify_image.create_graph()
        self._cv_bridge = CvBridge()

        self.Sub =rospy.Subscriber('image', Image, Self.callback, queue_size=1)
        self._pub = rospy.Publisher('result', String, queue_size= 1)
        self.score_threshold = rospy.get_param('-score_threshold', 0.1)
        self.use_top_k =rospy.get_param('use_top_k', 5)
    def callback(self, image_msg):
        cv_image = self.cv_bridge.imagmsg_to_cv2(image_msg, "bgr8") # ros image to opencv image
        #copy from
        #classify_image.py
        image_data =cv2.imencode('.jpg', cv_image)[1].tostring()   #encoded to .jpg and stored as string in image_data
        #create graph from saved GraphDef
        softmax_tensor = self.session.graph.get_tensor_by_name('softmax:0') # softmax algorthm to fetch our tensor of interest
        predictions = self._session.run(
              softmax_tensor,{'DecodeJpeg/contents:0' : image_data}) # save the results in variable called predictions
        predictions = np.squeeze(predictions)   # numpy squeeze() function to convert returend array into singel array
         #create node 10--> English string lookup
        node_lookup = classify_image.NodeLookup()
        top_k = predictions.argsort()[-self.use_top_k:][::-1]
        for node_id in top_k:
            human_string = node_lookup.id_to_string(node_id)
            score = predictions[node_id]
            if score > self.score_threshold:
                rospy.loginfo('%s (score = %.5f)' % (human_string. score))
                self._pub.publish(human_string)             # publish the results

    def main(self):
        rospy.spin()

        if __name__ == '__main__':
            classify_image.setup_args()
            rospy.init_node('rostensorflow')
            tensor = RosTesnsorFlow()
            tensor.main()
