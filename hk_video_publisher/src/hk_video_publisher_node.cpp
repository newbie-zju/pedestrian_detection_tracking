//reference from:
//https://blog.csdn.net/ding977921830/article/details/75272384

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "HCNetSDK.h"
#include "PlayM4.h"
#include "LinuxPlayM4.h"
#include <fstream>
#include <unistd.h>

using namespace std;
using namespace cv;

//hk
string ip;
string username;
string passwd;
LONG lUserID;
HWND h = 0;
int nPort = -1;
list<cv::Mat> g_frameList;
static Mat dst;
pthread_mutex_t mutex;
int hk_video_height = 1080;
int hk_video_width = 1920;

void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void *nReserved1, LONG nReserved2) {
    long lFrameType = pFrameInfo->nType;
    if (lFrameType == T_YV12) {
        dst.create(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
        cv::Mat src(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (uchar *) pBuf);
        cv::cvtColor(src, dst, CV_YUV2BGR_YV12);
        cv::resize(dst,dst,Size(hk_video_width, hk_video_height));
        pthread_mutex_lock(&mutex);
        g_frameList.push_back(dst);
        pthread_mutex_unlock(&mutex);
    }
    usleep(1000);
}

void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *dwUser) {
    DWORD dRet;
    switch (dwDataType) {
        case NET_DVR_SYSHEAD:           //系统头
            if (!PlayM4_GetPort(&nPort))  //获取播放库未使用的通道号
            {
                break;
            }
            if (dwBufSize > 0) {
                if (!PlayM4_SetStreamOpenMode(nPort, STREAME_REALTIME)) {
                    dRet = PlayM4_GetLastError(nPort);
                    break;
                }
                if (!PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 1024 * 1024)) {
                    dRet = PlayM4_GetLastError(nPort);
                    break;
                }

                //设置解码回调函数 解码且显示
                if (!PlayM4_SetDecCallBackEx(nPort, DecCBFun, NULL, 0)) {
                    dRet = PlayM4_GetLastError(nPort);
                    break;
                }

                //打开视频解码
                if (!PlayM4_Play(nPort, h)) {
                    dRet = PlayM4_GetLastError(nPort);
                    break;
                }

                //打开音频解码, 需要码流是复合流
                if (!PlayM4_PlaySound(nPort)) {
                    dRet = PlayM4_GetLastError(nPort);
                    break;
                }
            }
            break;
            //usleep(500);
        case NET_DVR_STREAMDATA:  //码流数据
            if (dwBufSize > 0 && nPort != -1) {
                BOOL inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
                while (!inData) {
                    sleep(100);
                    inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
                    std::cerr << "PlayM4_InputData failed \n" << std::endl;
                }
            }
            break;
    }
}

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser) {
    char tempbuf[256] = {0};
    std::cout << "EXCEPTION_RECONNECT = " << EXCEPTION_RECONNECT << std::endl;
    switch (dwType) {
        case EXCEPTION_RECONNECT:    //预览时重连
            printf("pyd----------reconnect--------%d\n", int(time(NULL)));
            break;
        default:
            break;
    }
}

void *RunIPCameraInfo(void *) {
    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(1000, true);
    NET_DVR_SetLogToFile(3, (char *) string("./sdkLog").data());
    NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
    NET_DVR_SetRecvTimeOut(5000);
    lUserID = NET_DVR_Login_V30((char *) ip.data(), 8000, (char *) username.data(), (char *) passwd.data(),
                                &struDeviceInfo);

    NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);

    long lRealPlayHandle;
    NET_DVR_CLIENTINFO ClientInfo = {0};
    ClientInfo.lChannel = 1;
    ClientInfo.lLinkMode = 0;
    ClientInfo.hPlayWnd = 0;
    ClientInfo.sMultiCastIP = NULL;

    lRealPlayHandle = NET_DVR_RealPlay_V30(lUserID, &ClientInfo, g_RealDataCallBack_V30, NULL, 0);
    if (lRealPlayHandle < 0) {
        printf("pyd1---NET_DVR_RealPlay_V30 error\n");
    }
    sleep(-1);

    NET_DVR_Cleanup();
}


int main(int argc, char **argv) {
    //node
    ros::init(argc, argv, "hk_video_publisher_node");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    string pubic_topic;
    if (!ros::param::get("~pubic_topic", pubic_topic))pubic_topic = "/hk_video";
    image_transport::Publisher image_pub_ = it_.advertise(pubic_topic, 1);
    //camera
    bool show_image_flag;
    if (!ros::param::get("~show_image_flag", show_image_flag))show_image_flag = true;
    string WINDOW_NAME = "hk_org";
    if (show_image_flag)namedWindow(WINDOW_NAME, WINDOW_NORMAL);
    string video_height, video_width;
    if (!ros::param::get("~video_height", hk_video_height))hk_video_height = 1080;
    if (!ros::param::get("~video_width", hk_video_width))hk_video_width = 1920;
    double public_rate;
    if (!ros::param::get("~public_rate", public_rate))public_rate = 30.0;
    //hk
    if (!ros::param::get("~ip", ip))ip = "192.168.0.200";
    if (!ros::param::get("~username", username))username = "admin";
    if (!ros::param::get("~passwd", passwd))passwd = "zjucsc301";
    //pub
    ros::Rate rate(public_rate);
    Mat src;
    ros::Time rostime;
    cv_bridge::CvImage cvi;
    sensor_msgs::Image im;
    pthread_t getframe;

    pthread_mutex_init(&mutex, NULL);

    int ret = pthread_create(&getframe, NULL, RunIPCameraInfo, NULL);
    if (ret != 0) {
        printf("Create pthread error!\n");
        return -1;
    }

    while (ros::ok()) {
        pthread_mutex_lock(&mutex);
        if (g_frameList.size()) {
            src = g_frameList.back();

            rostime = ros::Time::now();
            cvi.header.stamp = rostime;
            cvi.header.frame_id = "";
            cvi.encoding = "bgr8";
            cvi.image = src;
            cvi.toImageMsg(im);

            if (show_image_flag) {
                imshow(WINDOW_NAME, src);
                waitKey(1);
            }

            image_pub_.publish(im);
//            ros::spinOnce();
            g_frameList.clear();
        }
        pthread_mutex_unlock(&mutex);
        rate.sleep();
    }
    return 0;
}  


