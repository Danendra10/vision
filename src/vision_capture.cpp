#include "vision/vision_main.h"

using namespace cv;
using namespace std;

//---Mutex
//========
boost::mutex mutex_field_raw_threshold;

//---Matrix
//==========
Mat field_raw_threshold;
Mat vision_capture_raw;
Mat vision_capture_rgb;
Mat vision_capture_yuv;

//---Timer
//========
ros::Timer tim_50_hz;

//---Prototypes
//=============
void CllbkTim50Hz(const ros::TimerEvent &event);

//---Publisher
//============
image_transport::Publisher pub_frame_bgr;
image_transport::Publisher pub_frame_yuv;
image_transport::Publisher pub_frame_field_yuv;

//---Vid Capture
//==============
VideoCapture cap("/home/danendra/Iris/jiancuk_raceto/src/vision/vid/percobaan.mkv");
// VideoCapture cap("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_7C21B0EF-video-index0");
// VideoCapture cap("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_B086D5DF-video-index0");

//---Thresh
//=========
int yMin = 93;
int yMax = 175;
int uMin = 127;
int uMax = 255;
int vMin = 0;
int vMax = 98;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_capture");

    ros::NodeHandle nh;
    ros::MultiThreadedSpinner MTS;
    image_transport::ImageTransport IT(nh);

    pub_frame_field_yuv = IT.advertise("/vision_yuv", 32);

    tim_50_hz = nh.createTimer(ros::Duration(0.02), CllbkTim50Hz);
    MTS.spin();
}

void CllbkTim50Hz(const ros::TimerEvent &event)
{
    //--Store the captured to matrix
    //==============================
    cap >> vision_capture_rgb;

    //---Flip
    //=======
    flip(vision_capture_rgb, vision_capture_rgb, 1);
    // vision_capture_rgb = vision_capture_rgb(Rect(0, 100, g_res_x, g_res_y * 0.65));
    resize(vision_capture_rgb, vision_capture_rgb, Size(g_res_y, g_res_x));
    rotate(vision_capture_rgb, vision_capture_rgb, ROTATE_90_COUNTERCLOCKWISE);

    cvtColor(vision_capture_rgb, vision_capture_yuv, CV_BGR2YUV);

    //---Publish Field Only
    //=====================
    sensor_msgs::ImagePtr msg_frame_field_yuv =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", vision_capture_yuv).toImageMsg();
    pub_frame_field_yuv.publish(msg_frame_field_yuv);

    // //---Field
    // //========
    // mutex_field_raw_threshold.lock();

    // mutex_field_raw_threshold.unlock();
}
    // namedWindow("view", WINDOW_AUTOSIZE);
    // createTrackbar("yMin", "view", &yMin, 255);
    // createTrackbar("yMax", "view", &yMax, 255);
    // createTrackbar("uMin", "view", &uMin, 255);
    // createTrackbar("uMax", "view", &uMax, 255);
    // createTrackbar("vMin", "view", &vMin, 255);
    // createTrackbar("vMax", "view", &vMax, 255);

    // Scalar lower(yMin, uMin, vMin);
    // Scalar upper(yMax, uMax, vMax);

    // inRange(vision_capture_yuv, lower, upper, field_raw_threshold);
