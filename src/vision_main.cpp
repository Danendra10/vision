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
Mat frame_bgr = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv_field = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv_ball = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat field_final_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat ball_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat frame_yuv_obs = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat raw_frame_yuv_obs = Mat::zeros(g_res_y, g_res_x, CV_8UC1);

uint16_t largest_contours_area = 0;
uint16_t largest_contours_index = 0;

//---Cam Vars
//============
float g_center_cam_x = g_res_x * 0.5 + 3;
float g_center_cam_y = g_res_y * 0.5 - 10;

//---Ball Vars
//============
/**
 * x have max val of 360
 * y have max val of 640
 * we could use uint16_t because it have max val of 65535
 * 9999 is when the ball is not found
 * */
vector<vector<Point>> ball_contours;
vector<Vec4i> ball_hierarchy;
uint16_t g_center_ball_x;
uint16_t g_center_ball_y;
uint8_t yuv_ball_thresh[6] = {19, 193, 0, 126, 151, 255};
uint8_t yuv_field_thresh[6] = {93, 175, 127, 255, 0, 98};
uint8_t g_counter_bola_in;
uint8_t g_counter_bola_out;
uint8_t status_bola;
float g_ball_on_frame_x;
float g_ball_on_frame_y;
_Float32x g_ball_on_frame_theta;

//---Obstacle vars
//================
vector<vector<Point>> obs_contours;
vector<Vec4i> obs_hierarchy;

unsigned short int obs_buffer[50], limit_buffer[50];
unsigned short int obs[50], limit[50];

uint16_t largest_threshold_size_obs;

//---Container
//============
Mat display_obs = Mat::zeros(Size(g_res_x, g_res_y), CV_8UC3);

//============================================================

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
    sub_frame_field_yuv = IT.subscribe("/vision_yuv", 32, CllbkSubFrameYuv);

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

    //---Ignore the center of Cam
    //===========================
    //make a filled circle

    // circle(frame_yuv_field, Point(g_center_cam_x, g_center_cam_y), 100, Scalar(255, 0, 0), -1);
    // rectangle(frame_yuv_field, Point(g_center_cam_x - 10, g_center_cam_y - 10), Point(g_center_cam_x + 10, g_center_cam_y + 10), Scalar(0, 0, 0), -1);

    // imshow("Field Threshold", frame_yuv_field);


    //==================
    //---Ball Threshold
    //==================

    Scalar lower_ball(yuv_ball_thresh[0], yuv_ball_thresh[2], yuv_ball_thresh[4]);
    Scalar upper_ball(yuv_ball_thresh[1], yuv_ball_thresh[3], yuv_ball_thresh[5]);

    inRange(frame_yuv, lower_ball, upper_ball, frame_yuv_ball);

    //=====================
    //---Obstacle Threshold
    //=====================

    mutex_field_raw_threshold.lock();
    mutex_field_final_threshold.lock();
    mutex_ball_threshold.lock();

    /**
     * Combine the field and ball together
     * Using or because we want to get the obstacle even if there is no ball
     * Using not to get it's inverse
     * Using and
     * */
    bitwise_or(frame_yuv_field, frame_yuv_ball, frame_yuv_obs);

    bitwise_not(frame_yuv_obs, frame_yuv_obs);

    // bitwise_and(frame_yuv_obs, frame_yuv_field, frame_yuv_obs);

    //---Ignore the center
    //====================
    circle(frame_yuv_obs, Point(g_center_cam_x, g_center_cam_y), 60, Scalar(0, 0, 0), -1);

    erode(frame_yuv_obs, frame_yuv_obs, getStructuringElement(MORPH_ELLIPSE, Size(11, 11)));

    dilate(frame_yuv_obs, frame_yuv_obs, getStructuringElement(MORPH_ELLIPSE, Size(11, 11)));
    imshow("Field Threshold", frame_yuv_obs);

    mutex_field_raw_threshold.unlock();
    mutex_field_final_threshold.unlock();
    mutex_ball_threshold.unlock();

    raw_frame_yuv_obs = frame_yuv_obs.clone();

    mutex_frame_bgr.lock();
    cvtColor(frame_yuv, frame_bgr, CV_YUV2BGR);
    mutex_frame_bgr.unlock();
    frame_yuv_obs = Scalar(0);

    //===========================
    //---Search for contours
    //===========================
    findContours(frame_yuv_ball, ball_contours, ball_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(raw_frame_yuv_obs, obs_contours, obs_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //---Get the biggest contour area for obs
    //=======================================
    for (unsigned int i = 0; i < obs_contours.size(); i++)
    {
        if (contourArea(obs_contours[i]) > largest_threshold_size_obs)
        {
            largest_threshold_size_obs = contourArea(obs_contours[i]);
            drawContours(frame_yuv_obs, obs_contours, i, Scalar(255), -1);
            drawContours(frame_bgr, obs_contours, i, Scalar(0, 255, 255), 1);
        }
    }

    //---Count the obstacle's distance
    //--- from the center of the cam
    //================================
    for (uint8_t angles = 0; angles < 60; angles++)
    {

        uint16_t pixel = 0;
        int16_t pixel_x = g_center_cam_x;
        int16_t pixel_y = g_center_cam_y;

        while (pixel_x >= 0 &&
               pixel_y >= 0 &&
               pixel_x < g_res_x &&
               pixel_y < g_res_y)
        {
            // draw small circle every iterations
            circle(frame_yuv, Point(pixel_x, pixel_y), 1, Scalar(0, 255, 0), 1);
            printf("pixel_x: %d, pixel_y: %d || pixel : %d || angles : %d\n", pixel_x, pixel_y, pixel, angles);
            // obs_buffer[i] = pixel_to_cm(pixel);
            if (frame_yuv_obs.at<unsigned char>(Point(pixel_x, pixel_y)) == 255 && pixel_x > g_center_cam_x + 50 && pixel_x < g_center_cam_x - 50 && pixel_y > g_center_cam_y + 50 && pixel_y < g_center_cam_y - 50)
            {
                circle(frame_yuv, Point(pixel_x, pixel_y), 2, Scalar(255, 255, 0), 1);
                break;
            }
            pixel++;
            pixel_x = g_center_cam_x + (pixel * cos(angles::from_degrees((float)angles * 6)));
            pixel_y = g_center_cam_y + (pixel * sin(angles::from_degrees((float)angles * 6)));
            // if (angles >= 0 && angles <= 15)
            // {
            //     pixel_x = g_center_cam_x + (pixel * cos(angles::from_degrees((float)angles * 6))) + 50;
            //     pixel_y = g_center_cam_y + (pixel * sin(angles::from_degrees((float)angles * 6))) - 50;
            // }
            // else
            // {
            //     pixel_x = g_center_cam_x + (pixel * cos(angles::from_degrees((float)angles * 6))) - 50;
            //     pixel_y = g_center_cam_y + (pixel * sin(angles::from_degrees((float)angles * 6))) - 50;
            // }
        }
        if (pixel_x < 0 || pixel_y < 0 || pixel_x >= g_res_x || pixel_y >= g_res_y)
            obs_buffer[angles] = 9999;
    }

    //====================================================================================

    //---Get the biggest contour area for ball
    //========================================
    uint16_t largest_area = 0;
    uint16_t largest_contour_index = 0;

    for (uint16_t i = 0; i < ball_contours.size(); i++)
    {
        if (contourArea(ball_contours[i], false) > largest_area)
        {
            largest_area = contourArea(ball_contours[i], false);
            largest_contour_index = i;
        }
    }

    //---Ball detected safety
    //=======================
    if (ball_contours.size())
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
    if (g_counter_bola_in > 100)
        status_bola = 1;
    else if (g_counter_bola_out > 300)
        status_bola = 0;

    //---Update Ball pos
    //==================
    if (status_bola)
    {
        //---Get the moments
        //==================
        vector<Moments> mu(ball_contours.size());
        for (uint16_t i = 0; i < ball_contours.size(); i++)
            mu[i] = moments(ball_contours[i], false);

        //---Get the mass centers
        //=======================
        vector<Point2f> mc(ball_contours.size());
        for (uint16_t i = 0; i < ball_contours.size(); i++)
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

        //---Update ball pos
        //==================
        g_center_ball_x = mc[largest_contour_index].x;
        g_center_ball_y = mc[largest_contour_index].y;

        //---Ball pos relative from mid of frame
        //======================================
        g_ball_on_frame_x = g_center_ball_x - g_center_cam_x;
        g_ball_on_frame_y = g_center_cam_y - g_center_ball_y;
        g_ball_on_frame_theta = atan2(g_ball_on_frame_y, g_ball_on_frame_x);

        //---Get Radius
        //=============
        static float ball_radius;
        minEnclosingCircle(ball_contours[largest_contour_index], mc[largest_contour_index], ball_radius);

        //---Draw ball pos
        //================
        circle(frame_bgr, mc[largest_contour_index], ball_radius, Scalar(0, 0, 255));

        // printf("Ball pos: %f | %f | %f\n", g_ball_on_frame_x, g_ball_on_frame_y, g_ball_on_frame_theta);
    }

    // imshow("view ball", frame_yuv_ball);
    // imshow("view field", frame_yuv_field);

    //---Vertical Line
    //================
    line(frame_bgr, Point(g_center_cam_x, 0), Point(g_center_cam_x, g_center_cam_y - 50), Scalar(0, 255, 0));
    line(frame_bgr, Point(g_center_cam_x, g_center_cam_y + 50), Point(g_center_cam_x, g_res_y), Scalar(0, 255, 0));

    //---Horizontal Line
    //==================
    line(frame_bgr, Point(0, g_center_cam_y), Point(g_center_cam_x - 50, g_center_cam_y), Scalar(0, 255, 0));
    line(frame_bgr, Point(g_center_cam_x + 50, g_center_cam_y), Point(g_res_x, g_center_cam_y), Scalar(0, 255, 0));

    //---Circle
    //=========
    circle(frame_bgr, Point(g_center_cam_x, g_center_cam_y), 50, Scalar(0, 255, 0));
    // imshow("frame bgr", frame_bgr);
    // imshow("view yuv", frame_yuv);
    waitKey(30);
}
