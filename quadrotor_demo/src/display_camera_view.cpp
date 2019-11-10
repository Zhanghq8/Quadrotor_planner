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
#include <queue>
#include <sensor_msgs/Image.h>


using namespace cv;
using namespace std;

#define image_width 480 // pixel
#define image_height 640 // pixel
#define m_image_width 6*tan(41.8/57.2958)*2*1000 //mm
#define m_image_height 14300 //mm
#define min_contours_area 20

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

        vector<vector<Point> > contours;
        vector<vector<Point2d> > contours_map;
        vector<Vec4i> hierarchy;
        vector<int> contours_index;
        priority_queue<pair<double, int>> pq;

        Mat binaryMat(ThresholdedImg.size(), ThresholdedImg.type());
        imshow("colorImg", ThresholdedImg);
        //Apply thresholding
        threshold(ThresholdedImg, ThresholdedImg, 150, 255, cv::THRESH_BINARY);
        Canny(ThresholdedImg, ThresholdedImg, thresh, thresh * 3, 3);
        // imshow("Thresholded Image", ThresholdedImg);
        // cv::waitKey(3); 
        // imshow("result image", ThresholdedImg);
        findContours(ThresholdedImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        if (contours.size() == 0)
        {
            cout << "target object missing" << endl;
        }
        else
        {
            Mat result(ThresholdedImg.size(), CV_8U, Scalar(0));
            Mat area(1, contours.size(), CV_32FC1);
            // float maxArea = area.at<float>(0);
            // int max = 0;
            cout << "contours number: " << contours.size() << endl;
            // for (int i = 0; i < (int)contours.size(); i++)
            // {
            //     area.at<float>(i) = contourArea(contours[i]);
            //     if (maxArea < area.at<float>(i))
            //     {
            //       maxArea = area.at<float>(i);
            //       max = i;
            //     }
            // }
            for (int i = 0; i < (int)contours.size(); i++)
            {   
                area.at<float>(i) = contourArea(contours[i]);
                double contours_area = area.at<float>(i);
                if (contours_area < min_contours_area) {
                    continue;
                }
                pq.push(make_pair(contourArea(contours[i]), i)); 
            }
            contours_index.clear();
            while (!pq.empty()) {
                int index = pq.top().second;
                contours_index.push_back(index);
                pq.pop();
            }
            for (int i=0; i<contours_index.size(); i++) {

                vector<Point2d> contours_p;
                Rect obstacle = boundingRect(Mat(contours[contours_index[i]]));
                // cup_size = cup.area();
                
                rectangle(ThresholdedImg, obstacle, Scalar(255), 1);

                cv::Point2f center;
                center.x = (obstacle.br().x - obstacle.tl().x)/2;
                center.y = (obstacle.br().y - obstacle.tl().y)/2;
                double ob_height = obstacle.height;
                double ob_width = obstacle.width;
                cout << "---------------------------" << endl;
                cout << "obstacle index: " << contours_index[i] << endl;
                cout << "obstacle height: " << ob_height << endl;
                cout << "obstacle width: " << ob_width << endl;
                cout << "obstacle center: (" << center.x << ", " << center.y << ")." << endl;
                cout << "obstacle pixel vector: " << endl;
                image2map(contours[contours_index[i]], contours_p);
                contours_map.push_back(contours_p);
                cout << "obstacle point in image: ";
                for (auto cont : contours[i]) {
                    cout << "[" << cont.x << "," << cont.y <<"]" << " ";
                }
                cout << "\n";
                cout << "obstacle point in map: ";
                for (auto cont : contours_p) {
                    cout << "[" << cont.x << "," << cont.y <<"]" << " ";
                }
                cout << "\n";
                double obs_area = area.at<float>(contours_index[i]) * m_image_height / image_height * m_image_width / image_width;
                cout << "obstacle area value in map:" << obs_area << endl;
            }



            // rect.area();     
            // rect.size();     
            // rect.tl();   
            // rect.br();      
            // rect.width();  
            // rect.height();  
            // vector<Moments> mu(contours.size());
            // mu[max] = moments(contours[max], false);

            // vector<Point2f> mc(contours.size());
            // mc[max] = Point2d(mu[max].m10 / mu[max].m00, mu[max].m01 / mu[max].m00);

            // imshow("result image", imgcolor);
            // cout << "x= " << (int)mc[max].x << "****" << "y= " << (int)mc[max].y << endl;
            imshow("Thresholded Image", ThresholdedImg);
            cv::waitKey(3); 

        // }
        // cout << "Cup area:" << cup_size << endl;
        // cv::waitKey(3);
        // // return imgThresholded;
        // return cup_size;
        }
    }

    void image2map(const vector<Point>& p, vector<Point2d>& p_map) {
        for (int i=0; i<p.size(); i++) {
            Point2d xy = pixel2coordinate(p[i]);
            p_map.push_back(xy);
        }
    }
    Point2d pixel2coordinate(Point p) {
        //image x -> map -y 
        //image y -> map -x
        double y_resolution = m_image_height / image_height;
        double x_resolution = m_image_width / image_width;
        Point2d xy;
        int x_sign  = 1, y_sign = 1;
        int x_p_length = p.x - image_height/2;
        int y_p_length = p.y - image_width/2;
        if (x_p_length > 0) {
            y_sign = -1;
        }
        if (y_p_length > 0) {
            x_sign = -1;
        }
        xy.x = abs(y_p_length) * x_sign * x_resolution + 400; // add camera offset
        xy.y = abs(x_p_length) * y_sign * y_resolution;
        return xy;
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
