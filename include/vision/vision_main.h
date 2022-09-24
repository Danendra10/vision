#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/package.h>

using namespace cv;

//---Camera Resolution
//======================
short int g_res_x = 360;
short int g_res_y = 640;

//---Prototypes
//======================
void LoadCameraConfig(std::string camera_name, std::string _param, short int _value);
void LoadConfig();


void LoadCameraConfig(std::string camera_name, std::string _param, short int _value)
{
    char _param_char[_param.length() + 1];
    system(_param_char);
}

void LoadConfig()
{
    
}



// std::string _path = ros::package::getPath("vision");
//     std::string _file = _path + "/config/" + camera_name + ".yaml";
//     std::cout << "Loading camera config file: " << _file << std::endl;
//     FileStorage fs(_file, FileStorage::READ);
//     if (!fs.isOpened())
//     {
//         std::cout << "Failed to open camera config file: " << _file << std::endl;
//         return;
//     }
//     fs[_param] >> _value;
//     fs.release();