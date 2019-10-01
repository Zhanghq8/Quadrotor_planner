#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <sensor_msgs/Image.h>


using namespace cv;
using namespace std;

#define image_size 640*480

class ImageConverter
{
    Mat colorImg, ThresholdedImg;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    public:
    ImageConverter(): it_(nh_)
    {
        image_sub_ = it_.subscribe("/drone1/downward_cam/camera/image", 1, &ImageConverter::imageCb, this);
        cv::namedWindow("Thresholded Image");
        cv::namedWindow("colorImg");
    }

    ~ImageConverter()
    {
        cv::destroyWindow("Thresholded Image");
        cv::destroyWindow("colorImg");
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        
        try
        {   
            // mono8
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
            colorImg = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // imshow("colorImg", colorImg);
        filter(colorImg);
        
        // cv::waitKey(3); 
    }

    void filter(const Mat& color_img)
    {   
        // ThresholdedImg = color_img;
        // for (int i=0; i<ThresholdedImg.rows; i++) {
        //     // uchar*data = image.ptr<uchar>(i);//得到第i行的首地址
        //     for (int j=0; j<ThresholdedImg.cols;j++)
        //     {
        //         cout << "image" << int(ThresholdedImg.ptr<uchar>(i)[j]);
        //     }
        // }
        // cout << endl;
        // imshow("Thresholded Image", ThresholdedImg);
        // cv::waitKey(3); 

        // int iLowH = 170;
        // int iHighH = 179;
        // int iLowS = 173;
        // int iHighS = 255;
        // int iLowV = 0;
        // int iHighV = 255;
        // int number=3;
        // float cup_size;

        // cvtColor( color_img, ThresholdedImg, CV_BGR2GRAY );
        ThresholdedImg = color_img;
        // imshow("Thresholded Image", ThresholdedImg);
        // cv::waitKey(3); 
        // cvtColor( color_img, ThresholdedImg, CV_BGR2GRAY );
        int thresh = 30;
        // Mat imgHSV;
        // vector<Mat> hsvSplit;
        // cvtColor(imgcolor, imgHSV, COLOR_BGR2HSV); 

        // split(imgHSV, hsvSplit);
        // equalizeHist(hsvSplit[2],hsvSplit[2]);
        // merge(hsvSplit,imgHSV);
        // Mat imgThresholded;

        // inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        // ThresholdedImg = color_img;
        // Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        // morphologyEx(ThresholdedImg, ThresholdedImg, MORPH_OPEN, element);
        // morphologyEx(ThresholdedImg, ThresholdedImg, MORPH_CLOSE, element);

        GaussianBlur(ThresholdedImg, ThresholdedImg, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
        blur(ThresholdedImg, ThresholdedImg, Size(3, 3));

        // Mat canny_output;
        // // vector<vector<Point> > contours;
        // // vector<Vec4i> hierarchy;
        Mat binaryMat(ThresholdedImg.size(), ThresholdedImg.type());
        imshow("colorImg", ThresholdedImg);
        //Apply thresholding
        threshold(ThresholdedImg, ThresholdedImg, 150, 155, cv::THRESH_BINARY);
        Canny(ThresholdedImg, ThresholdedImg, thresh, thresh * 3, 3);
        imshow("Thresholded Image", ThresholdedImg);
        cv::waitKey(3); 
        // imshow("result image", ThresholdedImg);
        // findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // if (contours.size() == 0)
        // {
        //     cout << "target object missing" << endl;
        //     cup_size = 0;
        // }
        // else
        // {
        //     Mat result(imgThresholded.size(), CV_8U, Scalar(0));
        //     Mat area(1, contours.size(), CV_32FC1);
        //     float maxArea = area.at<float>(0);
        //     int max = 0;
        //     for (int i = 0; i < (int)contours.size(); i++)
        //     {
        //         area.at<float>(i) = contourArea(contours[i]);
        //         if (maxArea < area.at<float>(i))
        //         {
        //           maxArea = area.at<float>(i);
        //           max = i;
        //         }
        //     }

        //     Rect cup = boundingRect(Mat(contours[max]));
        //     cup_size = cup.area();
            
        //     // rectangle(imgcolor, r, Scalar(255), 1);

        //     // vector<Moments> mu(contours.size());
        //     // mu[max] = moments(contours[max], false);

        //     // vector<Point2f> mc(contours.size());
        //     // mc[max] = Point2d(mu[max].m10 / mu[max].m00, mu[max].m01 / mu[max].m00);

        //     // imshow("result image", imgcolor);
        //     // cout << "x= " << (int)mc[max].x << "****" << "y= " << (int)mc[max].y << endl;
        // }
        // cout << "Cup area:" << cup_size << endl;
        // cv::waitKey(3);
        // // return imgThresholded;
        // return cup_size;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_view");
    ros::NodeHandle nh;
    ImageConverter ic;
    ros::spin();
    return 0;
}
