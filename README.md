# pedestrian_detestion_tracking

for wen ling

## setup

<font color='red'>**need some modifications below**</font>

### ros
    gedit ~/.bashrc
    # add
    source ~/ros_wl/devel/setup.bash

### hk_video_publisher
    gedit ~/.bashrc
    # add
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zj/ros_wl/src/pedestrian_detection_tracking/hk_video_publisher/lib:/home/zj/ros_wl/src/pedestrian_detection_tracking/hk_video_publisher/lib/HCNetSDKCom

### pedstrian_detection

**tensorflow**

    sudo pip install https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-1.4.0-cp27-none-linux_x86_64.whl

**lib**

    gedit ~/.bashrc
    # add
    export PYTHONPATH=/home/zj/ros_wl/src/pedestrian_detection_tracking/pedstrian_detection/lib/object_detection:$PYTHONPATH
    export PYTHONPATH=/home/zj/ros_wl/src/pedestrian_detection_tracking/pedstrian_detection/lib:$PYTHONPATH

**detect_image.py**

    OBJECT_DETECTION_PATH = '/home/zj/ros_wl/src/pedestrian_detection_tracking/pedstrian_detection/lib/object_detection'

**pedstrian_detection_node.py**

    OBJECT_DETECTION_PATH = '/home/zj/ros_wl/src/pedestrian_detection_tracking/pedstrian_detection/lib/object_detection'
    PATH_TO_CKPT = '/home/zj/ros_wl/src/pedestrian_detection_tracking/pedstrian_detection/model/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'