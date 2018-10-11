# pedstrian_detection

Follow `Tensorflow Object Detection API` in `tensorflow/models` to prepare environment before running.

Tensorflow Object Detection API: https://github.com/tensorflow/models/blob/master/research/object_detection .

**Note**: Protobuf should be 2.6 version, you can get version by `protoc --version`, upgrade Protobuf by website: http://blog.csdn.net/sparkexpert/article/details/73456767 .

## detect_image.py

Contain a class of object_detection named DetectImage.

**Note**: Before using DetectImage, `OBJECT_DETECTION_PATH`, `PATH_TO_CKPT`, `PATH_TO_LABELS` and `NUM_CLASSES` should be modified.

## object_detection_ros_py.py

**Note**:

1. Some parameters need to modify, you can find them all in DetectVideo class. 
2. Run the node by `Python object_detection_ros_py.py`, `rosrun` or `roslaunch`
3. Another rosnode that can publish image stream is needed, for example, video_publisher_py: https://github.com/jzhugithub/video_publisher_py .
