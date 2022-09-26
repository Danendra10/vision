#include <angles/angles.h>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt16MultiArray.h>
#include <vector>

#include "vision/vision_main.h"

//---Timer
//======================
ros::Timer timer_50hz;

//-----Subscriber
image_transport::Subscriber sub_frame_bgr;
image_transport::Subscriber sub_frame_yuv;
image_transport::Subscriber sub_field_raw_threshold;
image_transport::Subscriber sub_field_final_threshold;
image_transport::Subscriber sub_ball_threshold;

//-----Publisher
image_transport::Publisher pub_raw_threshold;
image_transport::Publisher pub_final_threshold;
image_transport::Publisher pub_display_out;

ros::Publisher pub_obs;

boost::mutex mutex_frame_bgr;
boost::mutex mutex_frame_yuv;
boost::mutex mutex_field_raw_threshold;
boost::mutex mutex_field_final_threshold;
boost::mutex mutex_ball_threshold;

Mat frame_bgr = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat field_raw_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat field_final_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat ball_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat obstacle_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);

int yMin = 0;
int uMin = 0;
int vMin = 0;

int yMax = 255;
int uMax = 255;
int vMax = 255;



void CllbkSubFrameBgr(const sensor_msgs::ImageConstPtr &msg);
void CllbkSubFrameYuv(const sensor_msgs::ImageConstPtr &msg);
void CllbkTim50Hz(const ros::TimerEvent &event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_main");

    ros::NodeHandle nh;
    ros::MultiThreadedSpinner MTS;
    image_transport::ImageTransport IT(nh);

    timer_50hz = nh.createTimer(ros::Duration(0.02), CllbkTim50Hz);

    //---Subscriber
    //======================
    // sub_frame_bgr = IT.subscribe("/vision_frame_bgr", 32, CllbkSubFrameBgr);
    sub_frame_yuv = IT.subscribe("/vision_frame_yuv", 32, CllbkSubFrameYuv);
    // sub_field_raw_threshold = IT.subscribe("vision_field_raw_threshold", 32, &field_raw_threshold_callback);
    // sub_field_final_threshold = IT.subscribe("vision_field_final_threshold", 32, &field_final_threshold_callback);
    // sub_ball_threshold = IT.subscribe("vision_ball_threshold", 32, &ball_threshold_callback);

    //---Publisher
    //======================
    // pub_raw_threshold = IT.advertise("vision_raw_threshold", 32);
    // pub_final_threshold = IT.advertise("vision_final_threshold", 32);
    // pub_display_out = IT.advertise("vision_display_out", 32);

    // pub_obs = nh.advertise<std_msgs::UInt16MultiArray>("vision_obs", 32);

    //---Timer
    //======================
    // timer_50hz = nh.createTimer(ros::Duration(0.02), &CllbkTim50Hz);

    // show the p_frame_bgr
    // while (ros::ok())
    // {
    //     cv::namedWindow("p_frame_bgr", cv::WINDOW_AUTOSIZE);
    //     cv::imshow("p_frame_bgr", frame_bgr);
    // }

    MTS.spin();
}

// void CllbkSubFrameBgr(const sensor_msgs::ImageConstPtr &msg)
// {
    // try
    // {
    //     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //     cv::waitKey(30);
    // }
    // catch (cv_bridge::Exception &e)
    // {
    //     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    // }
//     mutex_frame_bgr.lock();
//     frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
//     mutex_frame_bgr.unlock();
// }

void CllbkSubFrameYuv(const sensor_msgs::ImageConstPtr &msg)
{
    // try
    // {
    //     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //     cv::waitKey(30);
    // }
    // catch (cv_bridge::Exception &e)
    // {
    //     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    // }
    mutex_frame_yuv.lock();
    frame_yuv = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    mutex_frame_yuv.unlock();
}

void CllbkTim50Hz(const ros::TimerEvent &event)
{
    // create trackbar to control YUV
    cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("yMin", "view", &yMin, 255);
    cv::createTrackbar("yMax", "view", &yMax, 255);
    cv::createTrackbar("uMin", "view", &uMin, 255);
    cv::createTrackbar("uMax", "view", &uMax, 255);
    cv::createTrackbar("vMin", "view", &vMin, 255);
    cv::createTrackbar("vMax", "view", &vMax, 255);

    cv::Scalar lower(yMin, uMin, vMin);
    cv::Scalar upper(yMax, uMax, vMax);

    cv::inRange(frame_yuv, lower, upper, field_raw_threshold);
    cv::imshow("view", field_raw_threshold);
    cv::imshow("view 1", frame_yuv);
    cv::waitKey(30);
}