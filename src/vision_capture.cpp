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
    
    image_transport::Publisher pub_frame_bgr = IT.advertise("vision_frame_bgr", 32);
    image_transport::Publisher pub_frame_yuv = IT.advertise("vision_frame_hsv", 32);

    Mat myImage;                     
    VideoCapture cap(0);         
    if (!cap.isOpened())
    { 
        cout << "No video stream detected" << endl;
        system("pause");
        return -1;
    }
    while (ros::ok())
    { 
        cap >> myImage;
        if (myImage.empty())
        { 
            break;
        }
        // imshow("Video Player", myImage); 
        char c = (char)waitKey(25);      
        if (c == 27)
        { 
            break;
        }
        flip(myImage, myImage, 1);
        resize(myImage, myImage, Size(g_res_y, g_res_x));
        rotate(myImage, myImage, ROTATE_90_CLOCKWISE);

        imshow("Video Player", myImage);

        Mat frame_bgr;
        frame_bgr = myImage.clone();
        sensor_msgs::ImagePtr msg_frame_bgr =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_bgr).toImageMsg();
        pub_frame_bgr.publish(msg_frame_bgr);

        Mat frame_yuv;
        cvtColor(myImage, frame_yuv, CV_BGR2YUV);
        sensor_msgs::ImagePtr msg_frame_yuv =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_yuv).toImageMsg();
        pub_frame_yuv.publish(msg_frame_yuv);

        ros::spinOnce();
    }
    ros::spin();  

    cap.release();
    return 0;
}