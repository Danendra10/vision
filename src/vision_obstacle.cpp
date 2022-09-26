/**
 * TODO:
 *  - get ball [v]
 *  - get field []
 *  - get obs []
 * */



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
Mat frame_yuv_copy = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat ball_raw_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat field_final_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat ball_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat obstacle_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);

int yMin = 19;
int uMin = 0;
int vMin = 151;

int yMax = 193;
int uMax = 126;
int vMax = 255;

int yMin_Ball = 19;
int uMin_Ball = 0;
int vMin_Ball = 151;

int yMax_Ball = 193;
int uMax_Ball = 126;
int vMax_Ball = 255;

int yMin_Line = 19;
int uMin_Line = 0;
int vMin_Line = 151;

int yMax_Line = 193;
int uMax_Line = 126;
int vMax_Line = 255;

uint16_t largest_contours_area = 0;
uint16_t largest_contours_index = 0;

void CllbkSubFrameBgr(const sensor_msgs::ImageConstPtr &msg);
void CllbkSubFrameYuv(const sensor_msgs::ImageConstPtr &msg);
void CllbkTim50Hz(const ros::TimerEvent &event);

RNG rng(12345);

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

    cv::inRange(frame_yuv, lower, upper, ball_raw_threshold);

    Mat canny_out;
    Canny(ball_raw_threshold, canny_out, 100, 200, 3);

    std::vector<std::vector<Point>> contours;
    findContours(canny_out, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<Point>> contours_poly(contours.size());
    std::vector<Rect> boundRect(contours.size());
    std::vector<Point2f> centers(contours.size());
    std::vector<float> radius(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = boundingRect(contours_poly[i]);
        minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }
    Mat drawing = Mat::zeros(canny_out.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(drawing, contours_poly, (int)i, color);
        circle(drawing, centers[i], (int)radius[i], color, 2);
    }
    imshow("Contours", drawing);
    cv::waitKey(30);

    // std::vector<std::vector<Point>> contours;
    // std::vector<Vec4i> hierarchy;
    // findContours(field_raw_threshold, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // print contour
    //  for (int i = 0; i < contours.size(); i++)
    //  {
    //      for (int j = 0; j < contours[i].size(); j++)
    //      {
    //          std::cout << contours[i][j] << std::endl;
    //      }
    //  }

    // for(int i = 0; i < contours.size(); i++)
    // {
    //     if(contourArea(contours[i]) > largest_contours_area)
    //     {
    //         largest_contours_area = contourArea(contours[i]);
    //         largest_contours_index = i;
    //     }
    // }
    // static Point2f ball_center;
    // static float ball_radius;
    // minEnclosingCircle(contours[largest_contours_index], ball_center, ball_radius);

    // circle(frame_yuv, ball_center, ball_radius, Scalar(0, 0, 255), 2, 8, 0);

    // cv::imshow("view", field_raw_threshold);
    // cv::imshow("view 1", frame_yuv);
    // cv::waitKey(30);
}