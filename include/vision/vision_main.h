#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/package.h>

#include "vision/config.h"

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

// float pixel_to_cm(float _pixel)
// {
//     // Nilai regresi diambil dari config/vision.yaml
//     double result = 0;
//     for (int i = 0; i < regresi.size(); i++)
//         result += (regresi[i] * pow(_pixel, (double)i));

//     return result;
// }

// void LoadConfig(uint8_t status, uint8_t *yuv_thresh)
// {
//     Config cfg;
//     // field
//     cfg.load("/home/danendra/Iris/jiancuk_raceto/cfg/IRIS1.yaml");

//     // field
//     // if (status == 0)
//     // {
//     // }
//     // ball
//     if (status == 1)
//     {
//         cfg.parseMapBegin("ball");
//         cfg.parseKeyValue("y_min", &yuv_thresh[0]);
//         cfg.parseKeyValue("y_max", &yuv_thresh[1]);
//         cfg.parseKeyValue("u_min", &yuv_thresh[2]);
//         cfg.parseKeyValue("u_max", &yuv_thresh[3]);
//         cfg.parseKeyValue("v_min", &yuv_thresh[4]);
//         cfg.parseKeyValue("v_max", &yuv_thresh[5]);
//         cfg.parseMapEnd();
//     }
// }

void LoadConfig()
{
    // Config cfg;
    // field
    // cfg.load("/home/danendra/Iris/jiancuk_raceto/cfg/IRIS1.yaml");

    // field
    // if (status == 0)
    // {
    // }
    // ball
    // if (status == 1)
    // {
        // cfg.parseMapBegin("ball");
        // cfg.parseKeyValue("y_min", &yuv_thresh[0]);
        // // cfg.parseKeyValue("y_max", &yuv_thresh[1]);
        // // cfg.parseKeyValue("u_min", &yuv_thresh[2]);
        // // cfg.parseKeyValue("u_max", &yuv_thresh[3]);
        // // cfg.parseKeyValue("v_min", &yuv_thresh[4]);
        // // cfg.parseKeyValue("v_max", &yuv_thresh[5]);
        // cfg.parseMapEnd();
    // }
}