#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from detect_image import DetectImage
from pdt_msgs.msg import BoundingBox
from pdt_msgs.msg import BoundingBoxes

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"


class DetectVideo(object):
    # parameters need to modify
    # node
    subscribed_topic = '/hk_video'
    pub_topic = '/pedstrian_bboxes'
    # video_output
    show_video_flag = True
    save_video_flag = False
    video_rate = 30.0
    video_output_path = os.path.join(os.path.abspath('../video_output'), 'out.avi')
    VIDEO_WINDOW_NAME = 'detect'
    # detect
    # Create DetectImage class
    OBJECT_DETECTION_PATH = '/home/zj/program/models/research/object_detection'
    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = '/home/zj/database_temp/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join(OBJECT_DETECTION_PATH, 'data/mscoco_label_map.pbtxt')
    NUM_CLASSES = 90


    # parameters do not need to modify
    # node
    # image_sub = rospy.Subscriber()
    # box_pub = rospy.Publisher()
    # video_output
    image_hight = -1
    image_width = -1
    video = 'VideoWriter'
    # frame
    frame_num = 1
    src = np.array([])
    dst = np.array([])
    cvi = CvBridge()
    # detect
    di = 'DetectImage'
    ready_flag = False

    def __init__(self):
        # node
        self.image_sub = rospy.Subscriber(self.subscribed_topic, Image, self.image_callback, queue_size=1)
        self.box_pub = rospy.Publisher(self.pub_topic, BoundingBoxes, queue_size=1)
        # video_output
        if self.show_video_flag:
            cv2.namedWindow(self.VIDEO_WINDOW_NAME, cv2.cv.CV_NORMAL)
        # detect
        self.di = DetectImage(self.PATH_TO_CKPT, self.PATH_TO_LABELS, self.NUM_CLASSES)
        self.ready_flag = True


    def __del__(self):
        cv2.destroyAllWindows()

    def image_callback(self, msg):
        if not self.ready_flag:
            return
        try:
            self.src = self.cvi.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        if self.frame_num == 1:
            self.image_hight, self.image_width, channels = self.src.shape
            self.video = cv2.VideoWriter(self.video_output_path, cv2.cv.CV_FOURCC('M', 'J', 'P', 'G'),
                                         int(self.video_rate), (int(self.image_width), int(self.image_hight)))
        self.frame_num += 1

        # detect
        # self.src_3 = cv2.resize(self.src_3,(160, 120))
        t1 = rospy.get_time()
        cv2.cvtColor(self.src, cv2.cv.CV_BGR2RGB, self.src)  # since opencv use bgr, but tensorflow use rbg
        self.dst, bboxs = self.di.run_detect(self.src, self.show_video_flag or self.save_video_flag)
        t2 = rospy.get_time()
        print("inference time: {}".format(t2-t1))

        # pub
        pub_msg = BoundingBoxes()
        pub_msg.header = msg.header
        pub_msg.bboxes = bboxs
        self.box_pub.publish(pub_msg)

        # save and show video_output
        if self.show_video_flag or self.save_video_flag:
            cv2.cvtColor(self.dst, cv2.cv.CV_RGB2BGR, self.dst)  # since opencv use bgr, but tensorflow use rbg
        if self.save_video_flag:
            self.video.write(self.dst)
        if self.show_video_flag:
            cv2.imshow(self.VIDEO_WINDOW_NAME, self.dst)
            cv2.waitKey(1)



if __name__ == '__main__':
    print('opencv: ' + cv2.__version__)
    rospy.init_node('pedstrian_detection_node', anonymous=False)
    mrd = DetectVideo()
    rospy.spin()
