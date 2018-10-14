#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import os
import tensorflow as tf
from matplotlib import pyplot as plt
import skimage.io
import sys

# Add object_detection to system path
OBJECT_DETECTION_PATH = os.path.join(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"lib"),"object_detection")
sys.path.append(OBJECT_DETECTION_PATH)

# Object detection imports
from utils import label_map_util
from utils import visualization_utils as vis_util
from pdt_msgs.msg import BoundingBox


def generate_bboxs(
        image,
        boxes,
        classes,
        scores,
        category_index,
        use_normalized_coordinates=False,
        max_boxes_to_draw=20,
        min_score_thresh=.5,
        agnostic_mode=False):
    """transform detection result to msg

    Args:
      image: uint8 numpy array with shape (img_height, img_width, 3)
      boxes: a numpy array of shape [N, 4]
      classes: a numpy array of shape [N]
      scores: a numpy array of shape [N] or None.  If scores=None, then
        this function assumes that the boxes to be plotted are groundtruth
        boxes and plot all boxes as black with no classes or scores.
      category_index: a dict containing category dictionaries (each holding
        category index `id` and category name `name`) keyed by category indices.
      use_normalized_coordinates: whether boxes is to be interpreted as
        normalized coordinates or not.
      max_boxes_to_draw: maximum number of boxes to visualize.  If None, draw
        all boxes.
      min_score_thresh: minimum score threshold for a box to be visualized
      agnostic_mode: boolean (default: False) controlling whether to evaluate in
        class-agnostic mode or not.  This mode will display scores but ignore
        classes.
    """

    bboxs = []

    if not max_boxes_to_draw:
        max_boxes_to_draw = boxes.shape[0]

    for i in range(min(max_boxes_to_draw, boxes.shape[0])):
        if scores[i] > min_score_thresh:
            if classes[i] in category_index.keys():
                class_name = category_index[classes[i]]['name']
                bbox = BoundingBox()
                ymin, xmin, ymax, xmax = boxes[i].tolist()
                bbox.ymin = int(ymin * image.shape[0])
                bbox.xmin = int(xmin * image.shape[1])
                bbox.ymax = int(ymax * image.shape[0])
                bbox.xmax = int(xmax * image.shape[1])
                bbox.ymin_normal = ymin
                bbox.xmin_normal = xmin
                bbox.ymax_normal = ymax
                bbox.xmax_normal = xmax
                bbox.score = scores[i]
                bbox.Class = class_name
                bboxs.append(bbox)
            else:
                class_name = 'N/A'
    return bboxs


class DetectImage(object):
    category_index = 'index'
    sess = 'sess'

    # graph input and output
    image_tensor = 'Tensor'
    # Each box represents a part of the image where a particular object was detected.
    detection_boxes = 'Tensor'
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    detection_scores = 'Tensor'
    detection_classes = 'Tensor'
    num_detections = 'Tensor'

    def __init__(self, PATH_TO_CKPT='.pb', PATH_TO_LABELS='.pbtxt', NUM_CLASSES=-1):
        '''
        Load category_index, load graph, run sess 
        :param PATH_TO_CKPT: 
            Path to frozen detection graph. This is the actual model that is used for the object detection.
        :param PATH_TO_LABELS: 
            List of the strings that is used to add correct label for each box.
        :param NUM_CLASSES: 
            Number of class for model to detect 
        '''
        if not os.path.exists(PATH_TO_CKPT):
            print('PATH_TO_CKPT not exist')
            return
        if not os.path.exists(PATH_TO_LABELS):
            print('PATH_TO_LABELS not exist')
            return

        # Set category_index
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        # Load a (frozen) Tensorflow model into memory.
        print('Load graph')
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        # Open graph and sess
        self.sess = tf.Session(graph=detection_graph)
        # Definite input and output Tensors for detection_graph
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    def __del__(self):
        self.sess.close()

    def run_detect(self, image_np, DRAW_BOX=True):
        '''
        run detect on a image
        :param image_np: image to detect
        :return: image with result, detection_boxes, detection_scores, detection_classes, num_detections
        '''

        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        # Visualization of the results of a detection.
        if DRAW_BOX:
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=8)

        bboxs = generate_bboxs(
            image_np,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True)

        return image_np, bboxs


if __name__ == '__main__':
    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = os.path.join(OBJECT_DETECTION_PATH, 'ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb')
    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join(OBJECT_DETECTION_PATH, 'data/mscoco_label_map.pbtxt')
    NUM_CLASSES = 90

    # Create DetectImage class
    di = DetectImage(PATH_TO_CKPT, PATH_TO_LABELS, NUM_CLASSES)

    # Size, in inches, of the output images.
    IMAGE_SIZE = (12, 8)
    PATH_TO_TEST_IMAGES_DIR = os.path.join(OBJECT_DETECTION_PATH, 'test_images')
    TEST_IMAGE_PATHS = [os.path.join(PATH_TO_TEST_IMAGES_DIR, 'image{}.jpg'.format(i)) for i in range(1, 3)]

    for image_path in TEST_IMAGE_PATHS:
        image_np = skimage.io.imread(image_path)
        image_np = di.run_detect(image_np)[0]

        plt.figure(figsize=IMAGE_SIZE)
        plt.imshow(image_np)
        plt.show()
