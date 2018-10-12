#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "kcftracker.hpp"
#include "pdt_msgs/TrackingDecisionResult.h"

using namespace cv;
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<pdt_msgs::TrackingDecisionResult, sensor_msgs::Image> syncPolicy;


class KcfTracker {
public:
    //node
    ros::NodeHandle nh_;
    //sub
    message_filters::Subscriber<pdt_msgs::TrackingDecisionResult> *decision_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;
    message_filters::Synchronizer<syncPolicy> *sync_;
    //pub
    ros::Publisher track_pub;
//    robot_kcftracker::KCFTracker msg_kcf;
    //video
    string VIDEO_WINDOW_NAME;
    bool show_video_flag;
    bool save_video_flag;
    double video_rate;
    VideoWriter video;
    string video_file_name;
    //frame
    Mat src_3;
    //kcf
    KCFTracker tracker;
    bool hog_flag;
    bool fixed_window_flag;
    bool multiscale_flag;
    bool lab_flag;
    Rect track_result;

    KcfTracker() {
        //sub
        string sub_decision_topic;
        if (!ros::param::get("~sub_decision_topic", sub_decision_topic))sub_decision_topic = "/tracking_decision";
        decision_sub = new message_filters::Subscriber<pdt_msgs::TrackingDecisionResult>(nh_, sub_decision_topic, 1);

        string sub_video_topic;
        if (!ros::param::get("~sub_video_topic", sub_video_topic))sub_video_topic = "/hk_video";
        image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, sub_video_topic, 1);

        // sync_
        sync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(100), *decision_sub, *image_sub);
        sync_->registerCallback(boost::bind(&KcfTracker::image_decision_callback, this, _1, _2));

        //pub
//        track_pub = nh_.advertise<robot_kcftracker::KCFTracker>("/kcf_track", 10);

        // video
        if (!ros::param::get("~show_video_flag", show_video_flag))show_video_flag = true;
        VIDEO_WINDOW_NAME = "kcf_track result";
        if (show_video_flag) {
            namedWindow(VIDEO_WINDOW_NAME, WINDOW_AUTOSIZE);
        }
        if (!ros::param::get("~save_video_flag", save_video_flag))save_video_flag = false;
        if (!ros::param::get("~video_rate", video_rate))video_rate = 10.0;
        if (!ros::param::get("~video_file_name", video_file_name))
            video_file_name = "/home/ubuntu/ros_my_workspace/src/robot_kcftracker/result/kcf.avi";
        //kcf
        if (!ros::param::get("~hog_flag", hog_flag))hog_flag = true;
        if (!ros::param::get("~fixed_window_flag", fixed_window_flag))fixed_window_flag = false;
        if (!ros::param::get("~multiscale_flag", multiscale_flag))multiscale_flag = true;
        if (!ros::param::get("~lab_flag", lab_flag))lab_flag = false;
        if (lab_flag == true)hog_flag = true;
        tracker = KCFTracker(hog_flag, fixed_window_flag, multiscale_flag, lab_flag);
    }

    ~KcfTracker() {
        destroyAllWindows();
    }

    void image_decision_callback(const pdt_msgs::TrackingDecisionResultConstPtr &decision_msg,
                                 const sensor_msgs::ImageConstPtr &image_msg) {
        // image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(src_3);

        // decision
        if (decision_msg->run)
        {
            if (decision_msg->begin) {
                //initializing kcf tracker
                // Using min and max of X and Y for groundtruth rectangle
                int init_xMin = decision_msg->init_box.xmin;
                int init_yMin = decision_msg->init_box.ymin;
                int init_xMax = decision_msg->init_box.xmax;
                int init_yMax = decision_msg->init_box.ymax;
                int init_width = init_xMax - init_xMin;
                int init_height = init_yMax - init_yMin;
                // First frame, give the groundtruth to the tracker
                tracker.init(Rect(init_xMin, init_yMin, init_width, init_height), src_3);

                if (show_video_flag || save_video_flag)
                    rectangle(src_3, Point(init_xMin, init_yMin), Point(init_xMax, init_yMax), Scalar(0, 255, 255), 1, 8);
                return;
            }

            //kcf update
            track_result = tracker.update(src_3);
            if (show_video_flag || save_video_flag)
                rectangle(src_3, Point(track_result.x, track_result.y),
                          Point(track_result.x + track_result.width, track_result.y + track_result.height),
                          Scalar(0, 255, 255), 1, 8);

//        //publish massage
//        msg_kcf.kcf_x = track_result.x;
//        msg_kcf.kcf_y = track_result.y;
//        msg_kcf.kcf_width = track_result.width;
//        msg_kcf.kcf_height = track_result.height;
//        msg_kcf.image_width = image_width;
//        msg_kcf.image_height = image_hight;
//        track_pub.publish(msg_kcf);
        }


        //save and show video
        if (save_video_flag)
            video << src_3;
        if (show_video_flag) {
            imshow(VIDEO_WINDOW_NAME, src_3);
            waitKey(1);
        }
    }
};

int main(int argc, char **argv) {
    //node
    ros::init(argc, argv, "kcf_tracker_node");//node name
    //class
    KcfTracker kt;//class initializing
    ros::spin();
    return 0;
}

