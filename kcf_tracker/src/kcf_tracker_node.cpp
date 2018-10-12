#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <strstream>
#include "kcftracker.hpp"

using namespace cv;
using namespace std;

string VIDEO_WINDOW_NAME = "kcf_track result";
//roi
Mat tmp_3;
Point2i pre_pt, end_pt, cur_pt;
bool select_end_flag = false;

void mouseSelectROI(int event, int x, int y, int flags, void *ustc) {
    cur_pt = Point2i(x, y);

    if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON)) {
        if (select_end_flag == false) {
            imshow(VIDEO_WINDOW_NAME, tmp_3);
            waitKey(1);
        }
        return;
    }
    if (event == CV_EVENT_LBUTTONDOWN) {
        pre_pt = Point2i(x, y);
    }
    if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)) {
        rectangle(tmp_3, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);
    }
    if (event == CV_EVENT_LBUTTONUP) {
        end_pt = cur_pt;
        rectangle(tmp_3, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);
        select_end_flag = true;
    }
    imshow(VIDEO_WINDOW_NAME, tmp_3);
    waitKey(1);
    return;
}

class RobotKcfTracker {
public:
    //node
    ros::NodeHandle nh_;
    ros::NodeHandle nh_image_param;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher track_pub;
    string subscribed_topic;
//    robot_kcftracker::KCFTracker msg_kcf;
    //video
    bool show_video_flag;
    bool save_video_flag;
    double video_rate;
    double image_hight;
    double image_width;
    double video_delay;
    VideoWriter video;
    string video_file_name;
    //frame
    int frame_num;
    Mat src_3, dst_3;
    //kcf
    KCFTracker tracker;
    bool hog_flag;
    bool fixed_window_flag;
    bool multiscale_flag;
    bool lab_flag;
    Rect track_result;
    float init_xMin;
    float init_yMin;
    float init_xMax;
    float init_yMax;
    float init_width;
    float init_height;
    bool kcf_init_end_flag;

    RobotKcfTracker() :
            it_(nh_),//intial it_
            nh_image_param("~") {
        //node
        if (!nh_image_param.getParam("subscribed_topic", subscribed_topic))subscribed_topic = "/hk_video";
        image_sub_ = it_.subscribe(subscribed_topic, 1, &RobotKcfTracker::imageCb, this);
//        track_pub = nh_.advertise<robot_kcftracker::KCFTracker>("/kcf_track", 10);
        //video
        if (!nh_image_param.getParam("show_video_flag", show_video_flag))show_video_flag = true;
        if (show_video_flag) {
            namedWindow(VIDEO_WINDOW_NAME);
        }
        if (!nh_image_param.getParam("save_video_flag", save_video_flag))save_video_flag = false;
        if (!nh_image_param.getParam("video_rate", video_rate))video_rate = 10.0;
        video_delay = 1000 / video_rate;
        if (!nh_image_param.getParam("video_file_name", video_file_name))
            video_file_name = "/home/ubuntu/ros_my_workspace/src/robot_kcftracker/result/kcf.avi";
        //frame
        frame_num = 1;
        //kcf
        if (!nh_image_param.getParam("hog_flag", hog_flag))hog_flag = true;
        if (!nh_image_param.getParam("fixed_window_flag", fixed_window_flag))fixed_window_flag = false;
        if (!nh_image_param.getParam("multiscale_flag", multiscale_flag))multiscale_flag = true;
        if (!nh_image_param.getParam("lab_flag", lab_flag))lab_flag = false;
        if (lab_flag == true)hog_flag = true;
        tracker = KCFTracker(hog_flag, fixed_window_flag, multiscale_flag, lab_flag);
        kcf_init_end_flag = false;
    }

    ~RobotKcfTracker() {
        destroyAllWindows();
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(src_3);
        src_3.copyTo(dst_3);
        //cout<<"rows"<<src_3.rows<<endl;
        //cout<<"cols"<<src_3.cols<<endl;

        //first frame
        if (frame_num == 1) {
            image_hight = src_3.rows;
            image_width = src_3.cols;
            if (save_video_flag)
                video = VideoWriter(video_file_name, CV_FOURCC('M', 'J', 'P', 'G'), video_rate,
                                    Size(image_width, image_hight));
            //set call back function for selecting inital target roi
            src_3.copyTo(tmp_3);
            setMouseCallback(VIDEO_WINDOW_NAME, mouseSelectROI, 0);
        }
        frame_num++;
        //imshow image for roi selecting
        if (select_end_flag == false) {
            imshow(VIDEO_WINDOW_NAME, tmp_3);
            waitKey(1);
            src_3.copyTo(tmp_3);
            return;
        }
        //initializing kcf tracker
        if (select_end_flag == true && kcf_init_end_flag == false) {
            // Using min and max of X and Y for groundtruth rectangle
            init_xMin = min(pre_pt.x, end_pt.x);
            init_yMin = min(pre_pt.y, end_pt.y);
            init_xMax = max(pre_pt.x, end_pt.x);
            init_yMax = max(pre_pt.y, end_pt.y);
            init_width = init_xMax - init_xMin;
            init_height = init_yMax - init_yMin;
            // First frame, give the groundtruth to the tracker
            tracker.init(Rect(init_xMin, init_yMin, init_width, init_height), src_3);
            rectangle(dst_3, Point(init_xMin, init_yMin), Point(init_xMax, init_yMax), Scalar(0, 255, 255), 1, 8);
            kcf_init_end_flag = true;
            return;
        }

        //kcf update
        track_result = tracker.update(src_3);
        rectangle(dst_3, Point(track_result.x, track_result.y),
                  Point(track_result.x + track_result.width, track_result.y + track_result.height), Scalar(0, 255, 255),
                  1, 8);

//        //publish massage
//        msg_kcf.kcf_x = track_result.x;
//        msg_kcf.kcf_y = track_result.y;
//        msg_kcf.kcf_width = track_result.width;
//        msg_kcf.kcf_height = track_result.height;
//        msg_kcf.image_width = image_width;
//        msg_kcf.image_height = image_hight;
//        track_pub.publish(msg_kcf);

        //save and show video
        if (save_video_flag) {
            video << dst_3;
        }
        if (show_video_flag) {
            imshow(VIDEO_WINDOW_NAME, dst_3);
            waitKey(1);
        }
    }
};

int main(int argc, char **argv) {
    //node
    ros::init(argc, argv, "kcf_tracker_node");//node name
    //class
    RobotKcfTracker rkt;//class initializing
    ros::spin();
    return 0;
}

