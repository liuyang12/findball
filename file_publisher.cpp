#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdlib>
#include <sstream>
#include <boost/format.hpp>

#define START_NUM 416
#define END_NUM 465     // 根据读入的文件名进行修改/home/young/文档/test_pics/ball/level1/clear/frame

boost::format g_format;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "file_publisher");
  ros::NodeHandle nh;

// Declare variables that can be modified by launch file or command line.
  std::string file;
  int frequency;

// Setting parameters with default values
  nh.setParam("file", std::string("/home/young/文档/test_pics/ball/level4/blurry/frame"));//
  nh.setParam("frequency", int(1));

// Getting the values of parameters if set from launch file or command line
  nh.getParam("file", file);
  nh.getParam("frequency", frequency);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image", 1);

  ros::Rate loop_rate(frequency);

  ROS_INFO("Ready to publish loaded image from directory:[%s] of frequency:[%d]", file.c_str(), frequency);

  int i;
  i = START_NUM;        // 根据读入的文件名进行修改/home/young/文档/test_pics/ball/level1/clear/frame

  g_format.parse("%s%04d.%s");
  while (nh.ok()) {
    if (i == END_NUM) i = START_NUM;// 根据读入的文件名进行修改/home/young/文档/test_pics/ball/level1/clear/frame
    std::string filename = (g_format %file % i % "jpg").str();
    /*std::stringstream ss;
    ss << file << i << ".bmp";
    ss >> filename;*/

    //sprintf(filename, "/home/chy/pictures/%d.bmp", i);
    ROS_INFO("Ready to publish loaded image from file:[%s]", filename.c_str());
// Loading the image, converting it to cv_bridge::CvImage type and to sensor_msgs::ImagePtr using .toImageMsg() 

//    cv::WImageBuffer3_b image( cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR) );

//    cv::Mat imageMat(image.Ipl());
    cv::Mat imageMat = cv::imread(filename.c_str(), 1);
    if(!imageMat.data)
    {
        continue;
    }
    cv_bridge::CvImage out_msg;
    out_msg.encoding = "bgr8";
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = imageMat;
//    out_msg.header.seq = i;
//    out_msg.header.frame_id = i;
    out_msg.header.stamp = ros::Time::now();

//    ci->header.seq = i;
//    ci->header.frame_id = i;
//    ci->header.stamp = out_msg.header.stamp;

    sensor_msgs::ImagePtr msg = out_msg.toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

    //image.ReleaseImage();
    //imageMat.release();
    i++;
  }
}
