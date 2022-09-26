#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/UInt16MultiArray.h>

#include "vision/vision_main.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_capture");

    ros::NodeHandle nh;
    image_transport::ImageTransport IT(nh);
    // Todo:
    // 1. Apa yang akan terjadi jika hanya ada yuv yang di pub
    // image_transport::Publisher pub_frame_bgr = IT.advertise("/vision_frame_bgr", 32);
    image_transport::Publisher pub_frame_yuv = IT.advertise("/vision_frame_yuv", 32);

    Mat vision_capture_rgb;
    VideoCapture cap("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_7C21B0EF-video-index0");
    if (!cap.isOpened())
    {
        cout << "No video stream detected" << endl;
        system("pause");
        return -1;
    }
    while (ros::ok())
    {
        cap >> vision_capture_rgb;

        if (vision_capture_rgb.empty())
            break;

        char c = (char)waitKey(25);

        if (c == 27)
            break;

        flip(vision_capture_rgb, vision_capture_rgb, 1);
        resize(vision_capture_rgb, vision_capture_rgb, Size(g_res_y, g_res_x));
        rotate(vision_capture_rgb, vision_capture_rgb, ROTATE_90_CLOCKWISE);

        // sensor_msgs::ImagePtr msg_frame_bgr =
        //     cv_bridge::CvImage(std_msgs::Header(), "bgr8", vision_capture_rgb).toImageMsg();
        // pub_frame_bgr.publish(msg_frame_bgr);

        Mat frame_yuv;
        cvtColor(vision_capture_rgb, frame_yuv, CV_BGR2YUV);
        sensor_msgs::ImagePtr msg_frame_yuv =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_yuv).toImageMsg();
        pub_frame_yuv.publish(msg_frame_yuv);

        ros::spinOnce();
    }
    ros::spin();

    cap.release();
    return 0;
}