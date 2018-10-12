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
    tracking_duration = 2.0

    # parameters do not need to modify
    # node
    # box_sub = rospy.Subscriber()
    # decision_pub = rospy.Publisher()
    begin_time = -1


    def __init__(self):
        # node
        self.box_sub = rospy.Subscriber(self.subscribed_topic, BoundingBoxes, self.box_callback, queue_size=1)
        self.decision_pub = rospy.Publisher(self.pub_topic, TrackingDecisionResult, queue_size=1)

    def box_callback(self, msg):
        box_person = self.get_person_box(msg.bboxes)
        has_person = False if len(box_person) < 1 else True
        overtime = False if rospy.get_time() - self.begin_time < self.tracking_duration else True

        pub_msg = TrackingDecisionResult()
        pub_msg.header = msg.header
        if has_person and overtime:
            pub_msg.run = True
            pub_msg.begin = True
            pub_msg.init_box = self.get_init_box(box_person)
            self.begin_time = rospy.get_time()
        elif has_person and not overtime:
            pub_msg.run = True
            pub_msg.begin = False
        elif not has_person and overtime:
            pub_msg.run = False
            pub_msg.begin = False
        elif not has_person and not overtime:
            pub_msg.run = True
            pub_msg.begin = False
        else:
            print("tracking decision, box_callback, pub bug")

        self.decision_pub.publish(pub_msg)


    def get_init_box(self, box_person):
        max_box = ""
        max_area = -1
        for box in box_person:
            area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
            if area > max_area:
                max_area = area
                max_box = box
        return max_box


    def get_person_box(self, bboxes):
        box_person = []
        for box in bboxes:
            if box.Class == "person":
                box_person.append(box)
        return box_person





if __name__ == '__main__':
    rospy.init_node('tracking_decision_node', anonymous=False)
    td = TrackingDecision()
    rospy.spin()
