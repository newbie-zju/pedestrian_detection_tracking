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
#include "pdt_msgs/BoundingBox.h"

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
    // video
    Mat src;
    string VIDEO_WINDOW_NAME;
    bool show_video_flag;
    bool save_video_flag;
    bool is_first_frame;
    int image_height;
    int image_width;
    double update_rate;
    VideoWriter video;
    string video_file_name;
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

        // pub
        string pub_track_topic;
        if (!ros::param::get("~pub_track_topic", pub_track_topic))pub_track_topic = "/track_box";
        track_pub = nh_.advertise<pdt_msgs::BoundingBox>(pub_track_topic, 10);

        // video
        if (!ros::param::get("~show_video_flag", show_video_flag))show_video_flag = true;
        VIDEO_WINDOW_NAME = "kcf_track result";
        if (show_video_flag)
            namedWindow(VIDEO_WINDOW_NAME, WINDOW_NORMAL);
        if (!ros::param::get("~save_video_flag", save_video_flag))save_video_flag = false;
        if (!ros::param::get("~update_rate", update_rate))update_rate = 10.0;
        if (!ros::param::get("~image_height", image_height))image_height = 540;
        if (!ros::param::get("~image_width", image_width))image_width = 960;
        if (!ros::param::get("~video_file_name", video_file_name))
            video_file_name = "/home/ubuntu/ros_my_workspace/src/robot_kcftracker/result/kcf.avi";
        is_first_frame = true;
        // kcf
        if (!ros::param::get("~hog_flag", hog_flag))hog_flag = true;
        if (!ros::param::get("~fixed_window_flag", fixed_window_flag))fixed_window_flag = false;
        if (!ros::param::get("~multiscale_flag", multiscale_flag))multiscale_flag = true;
        if (!ros::param::get("~lab_flag", lab_flag))lab_flag = false;
        if (lab_flag)hog_flag = true;
        tracker = KCFTracker(hog_flag, fixed_window_flag, multiscale_flag, lab_flag);
    }

    ~KcfTracker() {
        destroyAllWindows();
    }

    void run()
    {
        ros::Rate r(update_rate);
        while (ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
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
        cv_ptr->image.copyTo(src);

        if(is_first_frame && save_video_flag)
        {
            video = VideoWriter(video_file_name, CV_FOURCC('M', 'J', 'P', 'G'), update_rate, Size(image_width, image_height));
            is_first_frame = false;
        }

        // decision
        // pub
        pdt_msgs::BoundingBox track_box;
        track_box.header = decision_msg->header;
        track_box.xmin = -1;
        track_box.ymin = -1;
        track_box.xmax = -1;
        track_box.ymax = -1;
        track_box.xmin_normal = -1;
        track_box.ymin_normal = -1;
        track_box.xmax_normal = -1;
        track_box.ymax_normal = -1;

        if (decision_msg->run)
        {
            resize(src,src,Size(image_width, image_height));
            if (decision_msg->begin) {
                //initializing kcf tracker
                // Using min and max of X and Y for groundtruth rectangle
                int init_xMin = int(decision_msg->init_box.xmin_normal * image_width);
                int init_yMin = int(decision_msg->init_box.ymin_normal * image_height);
                int init_xMax = int(decision_msg->init_box.xmax_normal * image_width);
                int init_yMax = int(decision_msg->init_box.ymax_normal * image_height);
                int init_width = init_xMax - init_xMin;
                int init_height = init_yMax - init_yMin;
                // First frame, give the groundtruth to the tracker
                tracker.init(Rect(init_xMin, init_yMin, init_width, init_height), src);

                if (show_video_flag || save_video_flag)
                    rectangle(src, Point(init_xMin, init_yMin), Point(init_xMax, init_yMax), Scalar(0, 255, 255), 8, 8);
                return;
            }

            //kcf update
            track_result = tracker.update(src);
            if (show_video_flag || save_video_flag)
                rectangle(src, Point(track_result.x, track_result.y),
                          Point(track_result.x + track_result.width, track_result.y + track_result.height),
                          Scalar(0, 255, 255), 8, 8);

            // pub
            track_box.xmin = track_result.x;
            track_box.ymin = track_result.y;
            track_box.xmax = track_result.x + track_result.width;
            track_box.ymax = track_result.y + track_result.height;
            track_box.xmin_normal = double(track_box.xmin)/image_width;
            track_box.ymin_normal = double(track_box.ymin)/image_height;
            track_box.xmax_normal = double(track_box.xmax)/image_width;
            track_box.ymax_normal = double(track_box.ymax)/image_height;
        }

        // pub
        track_pub.publish(track_box);

        //save and show video
        if (save_video_flag)
            video << src;
        if (show_video_flag) {
            imshow(VIDEO_WINDOW_NAME, src);
            waitKey(1);
        }
    }
};

int main(int argc, char **argv) {
    //node
    ros::init(argc, argv, "kcf_tracker_node");//node name
    //class
    KcfTracker kt;//class initializing
    kt.run();
    return 0;
}

