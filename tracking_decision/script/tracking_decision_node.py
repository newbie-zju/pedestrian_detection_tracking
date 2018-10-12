#!/usr/bin/python
# -*- coding: utf-8 -*-

from pdt_msgs.msg import BoundingBoxes
import rospy

class TrackingDecision(object):
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('tracking_decision_node', anonymous=False)
    td = TrackingDecision()