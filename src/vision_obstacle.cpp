#include <angles/angles.h>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
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

unsigned char *p_frame_bgr = (unsigned char *)frame_bgr.data;

void CllbkSubFrameBgr(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_frame_bgr.lock();
    memcpy(&msg->data[0], p_frame_bgr, g_res_x * g_res_y * 3);
    mutex_frame_bgr.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_main");
    ros::NodeHandle nh;
    image_transport::ImageTransport IT(nh);

    //---Subscriber
    //======================
    sub_frame_bgr = IT.subscribe("vision_frame_bgr", 32, &CllbkSubFrameBgr);
    // sub_frame_yuv = IT.subscribe("vision_frame_yuv", 32, &CllbkSubFrameYuv);
    // sub_field_raw_threshold = IT.subscribe("vision_field_raw_threshold", 32, &field_raw_threshold_callback);
    // sub_field_final_threshold = IT.subscribe("vision_field_final_threshold", 32, &field_final_threshold_callback);
    // sub_ball_threshold = IT.subscribe("vision_ball_threshold", 32, &ball_threshold_callback);

    //---Publisher
    //======================
    pub_raw_threshold = IT.advertise("vision_raw_threshold", 32);
    pub_final_threshold = IT.advertise("vision_final_threshold", 32);
    pub_display_out = IT.advertise("vision_display_out", 32);

    pub_obs = nh.advertise<std_msgs::UInt16MultiArray>("vision_obs", 32);

    //---Timer
    //======================
    // timer_50hz = nh.createTimer(ros::Duration(0.02), &CllbkTim50Hz);

    // show the p_frame_bgr
    while (ros::ok())
    {
        cv::namedWindow("p_frame_bgr", cv::WINDOW_AUTOSIZE);
        cv::imshow("p_frame_bgr", p_frame_bgr);
    }

    ros::spin();
    return 0;
}