/**
 * TODO:
 *  - get ball [v]
 *  - get field [v]
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
image_transport::Subscriber sub_frame_field_yuv;
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

//---Matrix
//=========
Mat frame_yuv = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv_field = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv_ball = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat field_final_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat ball_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat obstacle_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);

uint16_t largest_contours_area = 0;
uint16_t largest_contours_index = 0;

//---Cam Vars
//============
float g_center_cam_x = g_res_x * 0.5;
float g_center_cam_y = g_res_y * 0.5;

//---Ball Vars
//============
/**
 * x have max val of 360
 * y have max val of 640
 * we could use uint16_t because it have max val of 65535
 * 9999 is when the ball is not found
 * */
uint16_t g_center_ball_x;
uint16_t g_center_ball_y;
uint8_t yuv_ball_thresh[6] = {19, 193, 0, 126, 151, 255};
uint8_t yuv_field_thresh[6] = {93,175,127,255,0,98};
uint8_t g_counter_bola_in;
uint8_t g_counter_bola_out;
uint8_t status_bola;
float g_ball_on_frame_x;
float g_ball_on_frame_y;
_Float32x g_ball_on_frame_theta;

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
    sub_frame_field_yuv = IT.subscribe("/vision_yuv", 32, CllbkSubFrameYuv);
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
    //     namedWindow("p_frame_bgr", WINDOW_AUTOSIZE);
    //     imshow("p_frame_bgr", frame_bgr);
    // }

    MTS.spin();
}

void CllbkSubFrameYuv(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        mutex_frame_yuv.lock();
        frame_yuv = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        mutex_frame_yuv.unlock();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CllbkTim50Hz(const ros::TimerEvent &event)
{   
    //================== 
    //---Field Threshold
    //==================
    Scalar lower_field(yuv_field_thresh[0], yuv_field_thresh[2], yuv_field_thresh[4]);
    Scalar upper_field(yuv_field_thresh[1], yuv_field_thresh[3], yuv_field_thresh[5]);

    inRange(frame_yuv, lower_field, upper_field, frame_yuv_field);

    //==================
    //---Ball Threshold
    //==================

    Scalar lower_ball(yuv_ball_thresh[0], yuv_ball_thresh[2], yuv_ball_thresh[4]);
    Scalar upper_ball(yuv_ball_thresh[1], yuv_ball_thresh[3], yuv_ball_thresh[5]);

    inRange(frame_yuv, lower_ball, upper_ball, frame_yuv_ball); 

    //=============================

    //---Search for ball contours
    //===========================
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(frame_yuv_ball, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //---Get the biggest contour area
    //===============================
    uint16_t largest_area = 0;
    uint16_t largest_contour_index = 0;

    for (uint16_t i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i], false) > largest_area)
        {
            largest_area = contourArea(contours[i], false);
            largest_contour_index = i;
        }
    }

    //---Ball detected safety
    //=======================
    if (contours.size())
    {
        g_counter_bola_in += 17;
        g_counter_bola_out = 0;
    }
    //---Lost the ball
    //================
    else
    {
        g_counter_bola_in = 0;
        g_counter_bola_out += 17;
    }

    //---Found the ball
    //=================
    if (g_counter_bola_in > 100) // iki asline pisan 100, safety ketika false detect
        status_bola = 1;
    // Bola dianggap hilang
    else if (g_counter_bola_out > 300)
        status_bola = 0;

    //---Update Ball pos
    //==================
    if (status_bola)
    {
        //---Get the moments
        //==================
        std::vector<Moments> mu(contours.size());
        for (uint16_t i = 0; i < contours.size(); i++)
            mu[i] = moments(contours[i], false);

        //---Get the mass centers
        //=======================
        std::vector<Point2f> mc(contours.size());
        for (uint16_t i = 0; i < contours.size(); i++)
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

        //---Update ball pos
        //==================
        g_center_ball_x = mc[largest_contour_index].x;
        g_center_ball_y = mc[largest_contour_index].y;

        //---Ball pos relative from mid of frame
        //======================================
        g_ball_on_frame_x = g_center_ball_x - g_center_cam_x;
        g_ball_on_frame_y = g_center_cam_y - g_center_ball_y;
        g_ball_on_frame_theta = atan2f32(g_ball_on_frame_y, g_ball_on_frame_x);

        //---Get Radius
        //=============
        static float ball_radius;
        minEnclosingCircle(contours[largest_contour_index], mc[largest_contour_index], ball_radius);        

        //---Draw ball pos
        //================
        circle(frame_yuv, mc[largest_contour_index], ball_radius, Scalar(0, 0, 255));

        printf("Ball pos: %f | %f | %f\n", g_ball_on_frame_x, g_ball_on_frame_y, g_ball_on_frame_theta);
    }

    imshow("view ball", frame_yuv_ball);
    imshow("view field", frame_yuv_field);
    imshow("view yuv", frame_yuv);
    waitKey(30);
}
