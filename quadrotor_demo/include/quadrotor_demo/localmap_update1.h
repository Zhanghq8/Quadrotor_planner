#ifndef LOCALMAP_UPDATE1_H_
#define LOCALMAP_UPDATE1_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <map>
#include <string>
#include <queue>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "quadrotor_demo/localmap.h"
#include "quadrotor_demo/obstacle_info.h"
#include "quadrotor_demo/obstacle.h"


using namespace cv;
using namespace std;

#define image_width 480 // pixel
#define image_height 640 // pixel
#define m_image_width 6*tan(41.8/57.2958)*2*1000 //10729mm
#define m_image_height 14300 //mm
#define min_contours_area 20

class LocalMapUpdate
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber update1Map_flag_sub_;
    ros::Subscriber current1pos_sub_;

    ros::Publisher update_complete_pub_;
    ros::Publisher localmap1_pub_;

    Mat colorImg, ThresholdedImg;
    string topic;
    bool updateFlag;
    bool updateCompleteFlag;
    vector<double> posvector;
public:
    LocalMapUpdate(ros::NodeHandle* nodehandle1, ros::NodeHandle* nodehandle2, string str);
    ~LocalMapUpdate();

    void initSub(); 
    void initPub();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void filter(const Mat& color_img);
   	void image2map(const vector<Point>& p, vector<Point2d>& p_map);
	Point2d pixel2coordinate(Point p);

	void update1mapflagCallback(const std_msgs::Bool& flag);
    void currentpos1Callback(const geometry_msgs::PoseStamped& odom1);
};


#endif /* LOCALMAP_UPDATE1_H_ */