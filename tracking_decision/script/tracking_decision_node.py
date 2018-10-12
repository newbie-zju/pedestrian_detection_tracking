#!/usr/bin/python
# -*- coding: utf-8 -*-

from pdt_msgs.msg import BoundingBoxes
from pdt_msgs.msg import TrackingDecisionResult
import rospy

class TrackingDecision(object):
    # parameters need to modify
    # node
    subscribed_topic = '/pedstrian_bboxes'
    pub_topic = '/tracking_decision'
    track_duration = 10.0

    # parameters do not need to modify
    # node
    # image_sub = rospy.Subscriber()
    # box_pub = rospy.Publisher()
    begin_time = -1


    def __init__(self):
        # node
        self.box_sub = rospy.Subscriber(self.subscribed_topic, BoundingBoxes, self.box_callback, queue_size=1)
        self.decision_pub = rospy.Publisher(self.pub_topic, TrackingDecisionResult, queue_size=1)

    def box_callback(self, msg):
        if msg.
        if rospy.get_time() - self.begin_time < self.track_duration:
            return




if __name__ == '__main__':
    rospy.init_node('tracking_decision_node', anonymous=False)
    td = TrackingDecision()
    rospy.spin()
