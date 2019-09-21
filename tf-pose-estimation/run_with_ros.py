#!/usr/bin/env python
import argparse
import logging
import sys
import time
import cv2
import json
# pose estimation libraries
from tf_pose import common
import numpy as np
from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import get_graph_path, model_wh
# ros libraries
import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
#from rospy_message_converter import message_converter


class image_converter:

    def __init__(self, model_name):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.pose_pub = rospy.Publisher("pose_topic",String, queue_size=5)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)
        #self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
        # from pose estimation code, dont know well what they do?
        self.logger = logging.getLogger('TfPoseEstimatorRun')
        self.logger.setLevel(logging.DEBUG)
        self.ch = logging.StreamHandler()
        self.formatter = logging.Formatter('[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s')
        self.ch.setFormatter(self.formatter)
        self.logger.addHandler(self.ch)
        self.model = model_name
        #self.resize = "432x368"
        #print self.model
        self.e = TfPoseEstimator(get_graph_path(self.model), target_size=(432,368))

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (w,h,channels) = cv_image.shape
        #print ("image retrieved")
        # code for running pose estimation on the image
        #image = cv2.resize(cv_image, (width,height))
        humans, scores = self.e.inference(cv_image, resize_to_default=(w > 0 and h > 0), upsample_size=4.0)
        #print ("---------Humans-------{}".format(humans))
        #print ("---------Scores-------{}".format(scores))
        #if cols > 60 and rows > 60 :
        #cv2.circle(cv_image, (50,50), 10, 255)

        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
        #print json.dumps(scores)
        try:
            pass
            self.pose_pub.publish(json.dumps(scores))
            #self.pose_pub.publish(std_msgs.msg.String(humans))
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='cmu',
                        help='cmu / mobilenet_thin / mobilenet_v2_large / mobilenet_v2_small')
    args = parser.parse_args()
    ic = image_converter(args.model)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

