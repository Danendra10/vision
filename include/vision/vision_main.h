#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/UInt16MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "vision/config.h"
#include "master/config.h"

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780

using namespace std;
using namespace cv;

//---Camera Resolution
//======================
uint16_t g_res_x = 360;
uint16_t g_res_y = 640;

//---Prototypes
//======================
void LoadCameraConfig(std::string camera_name, std::string _param, short int _value);
void LoadConfig();

void LoadCameraConfig(std::string camera_name, std::string _param, short int _value)
{
    char _param_char[_param.length() + 1];
    system(_param_char);
}

typedef struct
{
    int y_min;
    int y_max;
    int u_min;
    int u_max;
    int v_min;
    int v_max;
} d_yuv_th;