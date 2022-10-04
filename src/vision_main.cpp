/**
 * TODO:
 *  - get ball [v]
 *  - get field [v]
 *  - get obs [v]
 * obs masi bermasalah
 *
 * */
#include "vision/vision_main.h"
#include "master/Vision.h"
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
image_transport::Publisher pub_raw_frame;

image_transport::Publisher pub_ball_final_threshold_;
image_transport::Publisher pub_ball_display_out_;

image_transport::Publisher pub_field_final_threshold_;
image_transport::Publisher pub_field_display_out_;

ros::Publisher pub_vision;

boost::mutex mutex_frame_bgr;
boost::mutex mutex_frame_gray;
boost::mutex mutex_frame_yuv;
boost::mutex mutex_field_raw_threshold;
boost::mutex mutex_field_final_threshold;
boost::mutex mutex_obs_final_threshold;
boost::mutex mutex_ball_threshold;

//---Matrix
//=========
Mat field_gray;
Mat frame_yuv = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_bgr = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv_field = Mat::zeros(g_res_y, g_res_x, CV_8UC3);
Mat frame_yuv_ball = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat field_final_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat raw_field_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat ball_threshold = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat frame_yuv_obs = Mat::zeros(g_res_y, g_res_x, CV_8UC1);
Mat raw_frame_yuv_obs = Mat::zeros(g_res_y, g_res_x, CV_8UC1);

uint16_t largest_contours_area = 0;
uint16_t largest_contours_index = 0;

//---Cam Vars
//============
int8_t g_cam_offset_x = 10;
int8_t g_cam_offset_y = 20;

float g_center_cam_x = g_res_x * 0.5 + g_cam_offset_x;
float g_center_cam_y = g_res_y * 0.5 + g_cam_offset_y;

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

unsigned short int yuv_ball_thresh[6];
// uint8_t yuv_ball_thresh[6] = {0, 255, 0, 255, 159, 255};
uint8_t yuv_field_thresh[6] = {81, 255, 0, 140, 0, 114};
uint8_t g_counter_bola_in;
uint8_t g_counter_bola_out;
uint8_t ball_status;

/**
 * @brief index 0 - 3 is x, y, theta, and dist,
 * */
// float g_ball_on_frame[4];
// float g_ball_on_field[4];

float g_ball_x_on_frame;
float g_ball_y_on_frame;
float g_ball_theta_on_frame;
float g_ball_dist_on_frame;

float g_ball_dist_on_field;
float g_ball_theta_on_field;
float g_ball_x_on_field;
float g_ball_y_on_field;

//---Obstacle vars
//================
vector<vector<Point>> obs_contours;
vector<Vec4i> obs_hierarchy;

unsigned short int obs_buffer[60], limit_buffer[60];
unsigned short int obs[60], limit[60];

uint16_t largest_threshold_size_obs;

//---Container
//============
Mat display_obs = Mat::zeros(Size(g_res_x, g_res_y), CV_8UC3);
Mat display_field = Mat::zeros(Size(g_res_x, g_res_y), CV_8UC3);

//============================================================

//---Prototypes
//=============
void CllbkSubFrameBgr(const sensor_msgs::ImageConstPtr &msg);
void CllbkSubFrameYuv(const sensor_msgs::ImageConstPtr &msg);
void CllbkTim50Hz(const ros::TimerEvent &event);
float PixelToCm(float _pixel);

//---Iterator
//===========
uint16_t i;
uint8_t frame_angle;

//---Regression
//=============
vector<double> regresi = {
    2.1144972037850021e+003,
    -1.2529810711304636e+002,
    3.0395881121524386e+000,
    -3.8247974657217687e-002,
    2.6590340135171527e-004,
    -9.7049198872430191e-007,
    1.4626257536892665e-009,
};

//---Odom
//=======
_Float32 g_odom_x = 0;
_Float32 g_odom_y = 0;
_Float32 g_odom_theta = 90;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_main");

    ros::NodeHandle nh;
    ros::MultiThreadedSpinner MTS;
    image_transport::ImageTransport IT(nh);

    timer_50hz = nh.createTimer(ros::Duration(0.02), CllbkTim50Hz);

    LoadConfig();
    //---Subscriber
    //======================
    sub_frame_field_yuv = IT.subscribe("/vision_yuv", 32, CllbkSubFrameYuv);

    //---Publisher
    //============
    pub_raw_frame = IT.advertise("/vision_raw", 32);

    pub_field_final_threshold_ = IT.advertise("/vision_field_final_threshold", 32);
    pub_ball_final_threshold_ = IT.advertise("/vision_ball_final_threshold", 32);

    pub_vision = nh.advertise<master::Vision>("/vision_data", 16);

    // LoadConfig();

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
    // LoadConfig();
    //==================
    //---Field Threshold
    //==================
    Scalar lower_field(yuv_field_thresh[0], yuv_field_thresh[2], yuv_field_thresh[4]);
    Scalar upper_field(yuv_field_thresh[1], yuv_field_thresh[3], yuv_field_thresh[5]);

    inRange(frame_yuv, lower_field, upper_field, frame_yuv_field);

    erode(frame_yuv_field, frame_yuv_field, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));

    raw_field_threshold = frame_yuv_field.clone();

    mutex_frame_gray.lock();
    cvtColor(frame_yuv, frame_bgr, COLOR_YUV2BGR);
    cvtColor(frame_bgr, field_gray, COLOR_BGR2GRAY);
    mutex_frame_gray.unlock();

    //---Ignore the center of the cam
    //---We don't want the robot detect it as an obstacle
    //===================================================
    circle(field_gray, Point(g_center_cam_x, g_center_cam_y), 65, Scalar(0), -1);
    circle(field_gray, Point(g_center_cam_x, g_center_cam_y), 333, Scalar(0), 65);
    circle(display_field, Point(g_center_cam_x, g_center_cam_y), 65, Scalar(0, 0, 255), -1);

    bitwise_and(frame_yuv_field, field_gray, field_gray);
    adaptiveThreshold(field_gray, field_gray, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 99, 0);
    bitwise_or(field_gray, raw_field_threshold, raw_field_threshold);

    //==================
    //---Ball Threshold
    //==================

    Scalar lower_ball(yuv_ball_thresh[0], yuv_ball_thresh[2], yuv_ball_thresh[4]);
    Scalar upper_ball(yuv_ball_thresh[1], yuv_ball_thresh[3], yuv_ball_thresh[5]);

    inRange(frame_yuv, lower_ball, upper_ball, frame_yuv_ball);

    //=====================
    //---Obstacle Threshold
    //=====================
    /**
     * Combine the field and ball together
     * Using or because we want to get the obstacle even if there is no ball
     * Using not to get it's inverse
     * */
    mutex_field_raw_threshold.lock();
    mutex_field_final_threshold.lock();
    mutex_ball_threshold.lock();

    bitwise_or(raw_field_threshold, frame_yuv_ball, frame_yuv_obs);

    bitwise_not(frame_yuv_obs, frame_yuv_obs);

    mutex_field_raw_threshold.unlock();
    mutex_field_final_threshold.unlock();
    mutex_ball_threshold.unlock();

    //---Ignore the center
    //====================
    circle(frame_yuv_obs, Point(g_center_cam_x, g_center_cam_y), 65, Scalar(0, 0, 0), -1);

    erode(frame_yuv_obs, frame_yuv_obs, getStructuringElement(MORPH_ELLIPSE, Size(11, 11)));

    dilate(frame_yuv_obs, frame_yuv_obs, getStructuringElement(MORPH_ELLIPSE, Size(11, 11)));

    raw_frame_yuv_obs = frame_yuv_obs.clone();

    mutex_frame_bgr.lock();
    cvtColor(frame_yuv, frame_bgr, CV_YUV2BGR);
    display_obs = frame_bgr.clone();
    mutex_frame_bgr.unlock();

    frame_yuv_obs = Scalar(0);

    //===========================
    //---Search for contours
    //===========================

    //---Ball
    //=======
    findContours(frame_yuv_ball, ball_contours, ball_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //---Obs
    //======
    findContours(raw_frame_yuv_obs, obs_contours, obs_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //=======================================
    //---Get the biggest contour area for obs
    //=======================================
    for (i = 0; i < obs_contours.size(); i++)
    {
        if (contourArea(obs_contours[i]) > 1000)
        {
            drawContours(frame_yuv_obs, obs_contours, i, Scalar(255), -1);
            // drawContours(display_obs, obs_contours, i, Scalar(0, 255, 255), 1);
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
            obs_buffer[angles] = PixelToCm(pixel);
            if (frame_yuv_obs.at<unsigned char>(Point(pixel_x, pixel_y)) == 255)
            {
                // circle(display_obs, Point(pixel_x, pixel_y), 2, Scalar(255, 255, 0), 1);
                circle(frame_bgr, Point(pixel_x, pixel_y), 2, Scalar(255, 255, 0), 1);
                break;
            }
            pixel++;
            pixel_x = g_center_cam_x + (pixel * cos(angles::from_degrees((float)angles * 6)));
            pixel_y = g_center_cam_y - (pixel * sin(angles::from_degrees((float)angles * 6)));
        }
        if (pixel_x < 0 || pixel_y < 0 || pixel_x >= g_res_x || pixel_y >= g_res_y)
            obs_buffer[angles] = 9999;

        mutex_obs_final_threshold.lock();
        while (pixel_x >= 0 && pixel_y >= 0 && pixel_x < g_res_x && pixel_y < g_res_y)
        {
            limit_buffer[angles] = PixelToCm(pixel);

            if (frame_yuv_field.at<unsigned char>(Point(pixel_x, pixel_y)) == 0)
                break;

            pixel++;
            pixel_x = g_center_cam_x + (pixel * cos(angles::from_degrees((float)angles * 6)));
            pixel_y = g_center_cam_y - (pixel * sin(angles::from_degrees((float)angles * 6)));
        }
        if (pixel_x < 0 || pixel_y < 0 || pixel_x >= g_res_x || pixel_y >= g_res_y)
            limit_buffer[angles] = 9999;
        mutex_obs_final_threshold.unlock();
    }

    for (frame_angle = 0; frame_angle < 60; frame_angle++)
    {
        float field_angle = frame_angle * 6 + g_odom_theta - 90;
        while (field_angle >= 360)
            field_angle -= 360;
        while (field_angle < 0)
            field_angle += 360;

        obs_buffer[(int)(field_angle * 0.1667)] = obs_buffer[frame_angle];
        limit_buffer[(int)(field_angle * 0.1667)] = limit_buffer[frame_angle];
    }

    //========================================
    //---Get the biggest contour area for ball
    //========================================
    uint16_t largest_area = 0;
    uint16_t largest_contour_index = 0;

    for (i = 0; i < ball_contours.size(); i++)
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
        ball_status = 1;
    else if (g_counter_bola_out > 300)
        ball_status = 0;

    //---Update Ball pos
    //==================
    if (ball_status)
    {
        //---Get the moments
        //==================
        vector<Moments> mu(ball_contours.size());
        for (i = 0; i < ball_contours.size(); i++)
            mu[i] = moments(ball_contours[i], false);

        //---Get the mass centers
        //=======================
        vector<Point2f> mc(ball_contours.size());
        for (i = 0; i < ball_contours.size(); i++)
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

        //---Update ball pos
        //==================
        g_center_ball_x = mc[largest_contour_index].x;
        g_center_ball_y = mc[largest_contour_index].y;

        //---Ball pos relative from mid of frame
        //======================================
        g_ball_x_on_frame = g_center_ball_x - g_center_cam_x;
        g_ball_y_on_frame = g_center_cam_y - g_center_ball_y;
        g_ball_theta_on_frame = angles::to_degrees(atan2(g_ball_y_on_frame, g_ball_x_on_frame));

        g_ball_dist_on_frame = sqrt(g_ball_y_on_frame * g_ball_y_on_frame + g_ball_x_on_frame * g_ball_x_on_frame);
        g_ball_dist_on_field = PixelToCm(g_ball_dist_on_frame);

        g_ball_theta_on_field = g_ball_theta_on_frame + g_odom_theta - 90;
        while (g_ball_theta_on_field < -180)
            g_ball_theta_on_field += 360;
        while (g_ball_theta_on_field > 180)
            g_ball_theta_on_field -= 360;

        g_ball_x_on_field = g_odom_x + g_ball_dist_on_field *
                                           cos(angles::from_degrees(g_ball_theta_on_field));
        g_ball_y_on_field = g_odom_x + g_ball_dist_on_field *
                                           sin(angles::from_degrees(g_ball_theta_on_field));

        // printf("%f | %f | %f | %f\n", g_ball_x_on_field, g_ball_y_on_field, g_ball_theta_on_field, g_ball_theta_on_frame);

        //---Get Radius
        //=============
        static float ball_radius;
        minEnclosingCircle(ball_contours[largest_contour_index], mc[largest_contour_index], ball_radius);

        //---Draw ball pos
        //================
        circle(frame_bgr, mc[largest_contour_index], ball_radius, Scalar(0, 0, 255));

        // printf("Ball pos: %f | %f | %f\n", g_ball_x_on_frame, g_ball_y_on_frame, g_ball_theta_on_frame);
        // printf("Ball pos: %f | %f | %f\n", g_ball_x_on_field, g_ball_y_on_field, g_ball_theta_on_field);
        // printf("Ball on frame pos: %f | %f | %f\n", g_ball_x_on_frame, g_ball_y_on_frame, g_ball_theta_on_frame);
    }

    //---Draw Line
    //============
    line(frame_bgr, Point(g_center_cam_x, 0), Point(g_center_cam_x, g_res_y), Scalar(0, 255, 0), 1);
    line(frame_bgr, Point(0, g_center_cam_y), Point(g_res_x, g_center_cam_y), Scalar(0, 255, 0), 1);

    //---Circle
    //=========
    circle(frame_bgr, Point(g_center_cam_x, g_center_cam_y), 65, Scalar(0, 255, 0));

    //---Publishers
    //=============
    sensor_msgs::ImagePtr msg_raw_frame = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_bgr).toImageMsg();
    pub_raw_frame.publish(msg_raw_frame);

    sensor_msgs::ImagePtr msg_yuv_field = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_yuv_field).toImageMsg();
    pub_field_final_threshold_.publish(msg_yuv_field);

    sensor_msgs::ImagePtr msg_yuv_ball = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_yuv_ball).toImageMsg();
    pub_ball_final_threshold_.publish(msg_yuv_ball);

    master::Vision msg_vision;
    msg_vision.ball_on_frame_x = g_ball_x_on_frame;
    msg_vision.ball_on_frame_y = g_ball_y_on_frame;
    msg_vision.ball_on_frame_theta = g_ball_theta_on_frame;
    msg_vision.ball_on_frame_dist = g_ball_dist_on_frame;

    msg_vision.ball_on_field_x = g_ball_x_on_field;
    msg_vision.ball_on_field_y = g_ball_y_on_field;
    msg_vision.ball_on_field_theta = g_ball_theta_on_field;
    msg_vision.ball_on_field_dist = g_ball_dist_on_field;

    msg_vision.ball_status = ball_status;

    for (uint16_t angles = 0; angles < 360; angles += 6)
    {
        msg_vision.obs_on_field.push_back(obs[(int)(angles * 1.667)]);
        msg_vision.obs_limit.push_back(limit[(int)(angles * 1.667)]);
    }
    pub_vision.publish(msg_vision);

    imshow("BGR", frame_bgr);
    imshow("obs", display_obs);
    waitKey(30);
}

float PixelToCm(float _pixel)
{
    // Nilai regresi diambil dari config/vision.yaml
    double result = 0;
    for (int i = 0; i < regresi.size(); i++)
        result += (regresi[i] * pow(_pixel, (double)i));

    return result;
}

void LoadConfig()
{
    Config cfg;

    cfg.load("cfg.yaml");

    cfg.parseMapBegin("ball");
    cfg.parseKeyValue("y_min", &yuv_ball_thresh[0]);
    cfg.parseKeyValue("y_max", &yuv_ball_thresh[1]);
    cfg.parseKeyValue("u_min", &yuv_ball_thresh[2]);
    cfg.parseKeyValue("u_max", &yuv_ball_thresh[3]);
    cfg.parseKeyValue("v_min", &yuv_ball_thresh[4]);
    cfg.parseKeyValue("v_max", &yuv_ball_thresh[5]);
    cfg.parseMapEnd();
}