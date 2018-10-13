#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "some_method.h"
#include "parameter.h"


using namespace cv;
using namespace std;

class PedstrianDetectionOpencv
{
public:
    //node
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    string subscribed_topic;
    //HOG descriptor
    HOGDescriptor hog_descriptor_detect;
    //video
    string VIDEO_WINDOW_NAME;
    bool show_video_flag;
    bool save_result_video_flag;
    double video_rate;
    double image_hight;
    double image_width;
    VideoWriter result_video;
    string result_video_file_name;
    //frame
    bool is_first_frame;
    Mat src_3;
    vector<Rect> location_detect;
    vector<float> scores;

    PedstrianDetectionOpencv():
            it_(nh_)//intial it_
    {
        //node
        if(!ros::param::get("~subscribed_topic", subscribed_topic))subscribed_topic = "/hk_video";
        image_sub_ = it_.subscribe(subscribed_topic, 1, &PedstrianDetectionOpencv::imageCb, this);
        //hog
        hog_descriptor_detect.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        //getDaimlerPeopleDetector
        //video
        if(!ros::param::get("~show_video_flag", show_video_flag))show_video_flag = true;
        if(show_video_flag)
        {
            VIDEO_WINDOW_NAME="detect result";
            namedWindow(VIDEO_WINDOW_NAME);
        }
        if(!ros::param::get("~save_result_video_flag", save_result_video_flag))save_result_video_flag = false;
        if(!ros::param::get("~rate", video_rate))video_rate = 5.0;
        if(!ros::param::get("~result_video_file_name", result_video_file_name))result_video_file_name = "/home/ubuntu/ros_my_workspace/src/multirobot_detect/result/a544.avi";
        //frame
        is_first_frame = true;

    }

    ~PedstrianDetectionOpencv()
    {
        destroyAllWindows();
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(src_3);

        if(is_first_frame)
        {
            image_hight = src_3.rows;
            image_width = src_3.cols;
            result_video = VideoWriter(result_video_file_name, CV_FOURCC('M', 'J', 'P', 'G'), video_rate, Size(image_width, image_hight));
            is_first_frame = false;
        }

        //detect
        hog_descriptor_detect.detectMultiScale(src_3, location_detect, 0, Size(8,8), Size(0,0), 1.2, 2.0, false);
//        location_detect = resize_boxes(location_detect, src_3, detect_resize_rate);

        //Non-maximum suppression
//        scores = get_scores(src_3, location_detect, svm_detect, descriptor_dim_detect, WinSizeDetect, hog_descriptor_detect);
//        location_detect = non_maximum_suppression(location_detect, scores, SuppressionRate);

        //save and show video
        if(save_result_video_flag || show_video_flag)
        {
            for(int i=0; i<location_detect.size(); i++)
            {
                rectangle(src_3, location_detect[i], CV_RGB(0,0,255), 3);
            }
        }

        if(save_result_video_flag)
            result_video<<src_3;
        if(show_video_flag)
        {
            imshow(VIDEO_WINDOW_NAME, src_3);
            waitKey(1);
        }
    }
};

int main(int argc, char** argv)
{
    cout << "opencv version: "<<CV_VERSION << endl;
    ros::init(argc, argv, "pedstrian_detection_opencv_node");//node name
    double loop_rate;
    PedstrianDetectionOpencv pdo;//class initializing
    ros::spin();
    return 0;
}