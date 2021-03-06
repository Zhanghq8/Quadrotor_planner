#include "../include/quadrotor_demo/localmap_update.h"

LocalMapUpdate::LocalMapUpdate(ros::NodeHandle* nodehandle1, ros::NodeHandle* nodehandle2, string str): 
			nh_(*nodehandle1), it_(*nodehandle2), topic(str) {
    init();
    initSub();
    initPub();
}

LocalMapUpdate::~LocalMapUpdate() {

}

void LocalMapUpdate::init() {
    assert(topic.substr(0,6) == "/drone" && topic.substr(7) == "/downward_cam/camera/image" && "The input ros topic is doestn't match!");
    updateFlag = false;
    updateCompleteFlag = false;
    posvector = vector<double> (7, 0);
    string index;
    if (topic[6] == '1') {
        index = "1";
    } else if (topic[6] == '2') {
        index = "2";
    } else if (topic[6] == '3') {
        index = "3";
    } else if (topic[6] == '4') {
        index = "4";
    } else {
        assert(topic[6] >= '1' && topic[6] <= '4' && "The input ros topic is doestn't match!");
    }
    updateMapTopic = "/updatemap" + index;
    posTopic = "/drone" + index + "/ground_truth_to_tf/pose";
    updateCompleteTopic = "/update" + index + "_complete";
    localmapTopic = "/localmap" + index;
}

void LocalMapUpdate::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    image_sub_ = it_.subscribe(topic, 1, &LocalMapUpdate::imageCallback, this);
    updateMap_flag_sub_ = nh_.subscribe(updateMapTopic, 1, &LocalMapUpdate::updatemapflagCallback,this);
    currentpos_sub_ = nh_.subscribe(posTopic, 1, &LocalMapUpdate::currentposCallback,this);
}

void LocalMapUpdate::initPub() {
    // ROS_INFO("Initializing Publishers");
    update_complete_pub_ = nh_.advertise<std_msgs::Bool>(updateCompleteTopic, 1, true); 
    localmap_pub_ = nh_.advertise<quadrotor_demo::localmap>(localmapTopic, 1, true); 
}

void LocalMapUpdate::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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
    int x = 320-69 , y = 240-69+18, w = 138, h = 138;
    Rect ROI = Rect(x, y, w, h);
    Mat image_roi = colorImg(ROI);
    // imshow("roi_img", image_roi);
    // cout << "---------------------" << endl;
    // cout << "For rostopic: " << topic << endl;
    if (updateFlag) {
        // ros::Rate loop_rate(10);
        // loop_rate.sleep();
    	filter(colorImg);
        // loop_rate.sleep();
    }
}

void LocalMapUpdate::updatemapflagCallback(const std_msgs::Bool& flag) {
    updateFlag = flag.data;
}

void LocalMapUpdate::filter(const Mat& color_img) {      
    // grid number to coordinates in map
    // unordered_map<int, pair<int,int>> coordinates({ {0,{-1,1}}, {1,{0,1}}, {2, {1,1}} ,
    //                                                 {3,{-1,0}}, {4,{0,0}}, {5,{1,0}},
    //                                                 {6,{-1,-1}}, {7,{0,-1}}, {8,{1,-1}} });

    // grid number in image to grid number in map
    unordered_map<int, int> imageId2mapId({{6,0}, {3,1}, {0,2}, {7,3}, {4,4}, {1,5}, {8,6}, {5,7},{2,8}}); 
    //    image     map
    //     630      012
    //     741      345
    //     852      678

    // grid number vector
    vector<vector<bool>> occupancy(3, vector<bool> (3, false));
    vector<int> cover(9, 0);  
    map<int, bool> obstacle_info;

    // 400mm offset->18 pixels 
    // center: 320, 480
    // 3000mm -> 
    // create a roi image
    int x = 320-69 , y = 240-69+18, w = 138, h = 138;
    Rect ROI = Rect(x, y, w, h);
    Mat image_roi = color_img(ROI);
    // imshow("roi_img", image_roi);

    int width = image_roi.rows;  
    int height = image_roi.cols;  

    // create a copy of roi image, set the corner pixel as 255
    cv::Mat Imgcopy(width, height, CV_8UC1);  

    for (int i=0; i<image_roi.rows; i++) {
        for (int j=0; j<image_roi.cols;j++)
        {   
            int gray = image_roi.at<uchar>(i, j);
            if (i >= 0 && i < width/3 && j >= 0 && j < width/3) {
                if (gray == 0) {
                    cover[0]++;
                }
            }
            else if (i >= 0 && i < width/3 && j >= width/3 && j < 2*width/3) {
                if (gray == 0) {
                    cover[1]++;
                }
            }
            else if (i >= 0 && i < width/3 && j >= 2*width/3 && j < width) {
                if (gray == 0) {
                    cover[2]++;
                }
            }
            else if (i >= width/3 && i < 2*width/3 && j >= 0 && j < width/3) {
                if (gray == 0) {
                    cover[3]++;
                }
            }
            else if (i >= width/3 && i < 2*width/3 && j >= width/3 && j < 2*width/3) {
                if (gray == 0) {
                    cover[4]++;
                }
            }
            else if (i >= width/3 && i < 2*width/3 && j >= 2*width/3 && j < width) {
                if (gray == 0) {
                    cover[5]++;
                }
            }
            else if (i >= 2*width/3 && i < width && j >= 0 && j < width/3) {
                if (gray == 0) {
                    cover[6]++;
                }
            }
            else if (i >= 2*width/3 && i < width && j >= width/3 && j < 2*width/3) {
                if (gray == 0) {
                    cover[7]++;
                }
            }
            else  {
                if (gray == 0) {
                    cover[8]++;
                }
            }

            if (i <= 2 || i >= (image_roi.rows-2) || j <= 2 || j >= image_roi.cols-2) {
                Imgcopy.at<uchar>(i,j) = 255;
            }
            else {
                Imgcopy.at<uchar>(i,j) = gray;
            }
        }
    }

    for (int i=0; i<cover.size(); i++) {
        if (cover[i] >= 2116/2) {
            obstacle_info.insert({imageId2mapId[i], true});
        } else {
            obstacle_info.insert({imageId2mapId[i], false});
        }
    }

    quadrotor_demo::localmap localmap_msg;
    localmap_msg.xpos = posvector[0];
    localmap_msg.ypos = posvector[1];

    // TO DO: consider boundry of the map
    for (auto itr = obstacle_info.begin(); itr != obstacle_info.end(); itr++) {
        if (itr->first != 0 && itr->first != 2 && itr->first != 6 && itr->first != 8) {
            quadrotor_demo::obstacle_info obstacle_msg;
            obstacle_msg.id = itr->first;
            obstacle_msg.isobstacle = itr->second;
            // if (itr->second) {
            //     std::cout << "obstacle id: " << itr->first << std::endl;
            // }
            localmap_msg.obstacle_data.push_back(obstacle_msg);
        }
    }

    localmap_pub_.publish(localmap_msg);

    updateCompleteFlag = true;
    std_msgs::Bool ucf;
    ucf.data = updateCompleteFlag;
    update_complete_pub_.publish(ucf);
    updateFlag = false;
    std::cout << "===================================================" << std::endl;
    std::cout << "Measurement for sensor " << topic[6] << " finished." << std::endl;
    std::cout << "===================================================" << std::endl;
    // ThresholdedImg = Imgcopy;

    // // threshold for canny
    // int thresh = 30;

    // // Gaussian filter to smooth image
    // GaussianBlur(ThresholdedImg, ThresholdedImg, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
    // blur(ThresholdedImg, ThresholdedImg, Size(3, 3));

    // // OPEN and CLOSE operation to fill the narrow gap
    // // Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    // // morphologyEx(ThresholdedImg, ThresholdedImg, MORPH_OPEN, element);
    // // morphologyEx(ThresholdedImg, ThresholdedImg, MORPH_CLOSE, element);

    // vector<vector<Point> > contours;
    // vector<vector<Point2d> > contours_map;
    // vector<Vec4i> hierarchy;
    // vector<int> contours_index;
    // priority_queue<pair<double, int>> pq;

    // // imshow("ori_img", ThresholdedImg);
    // //Apply thresholding to convert image from gray to binary
    // threshold(ThresholdedImg, ThresholdedImg, 140, 255, cv::THRESH_BINARY);
    // // imshow("Binary Image", ThresholdedImg);

    // // canny edge detection
    // Canny(ThresholdedImg, ThresholdedImg, thresh, thresh * 3, 3);
    // // imshow("Canny Image", ThresholdedImg);

    // findContours(ThresholdedImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // if (contours.size() == 0)
    // {
    //     cout << "target object missing" << endl;
    // }
    // else
    // {
    //     Mat area(1, contours.size(), CV_32FC1);

    //     cout << "contours number: " << contours.size() << endl;

    //     for (int i = 0; i < (int)contours.size(); i++)
    //     {   
    //         area.at<float>(i) = contourArea(contours[i]);
    //         double contours_area = area.at<float>(i);
    //         if (contours_area < min_contours_area) {
    //             continue;
    //         }
    //         pq.push(make_pair(contourArea(contours[i]), i)); 
    //     }
    //     contours_index.clear();
    //     while (!pq.empty()) {
    //         int index = pq.top().second;
    //         contours_index.push_back(index);
    //         pq.pop();
    //     }
    //     for (int i=0; i<contours_index.size(); i++) {

    //         vector<Point2d> contours_p;
    //         Rect obstacle = boundingRect(Mat(contours[contours_index[i]]));
    //         // cup_size = cup.area();
            
    //         rectangle(ThresholdedImg, obstacle, Scalar(255), 1);

    //         cv::Point2f center;
    //         center.x = obstacle.tl().x + (obstacle.br().x - obstacle.tl().x)/2;
    //         center.y = obstacle.tl().y + (obstacle.br().y - obstacle.tl().y)/2;
    //         double ob_height = obstacle.height;
    //         double ob_width = obstacle.width;
    //         cout << "---------------------------" << endl;
    //         cout << "obstacle index: " << contours_index[i] << endl;
    //         cout << "obstacle height: " << ob_height << endl;
    //         cout << "obstacle width: " << ob_width << endl;
    //         cout << "obstacle center: (" << center.x << ", " << center.y << ")." << endl;
    //         cout << "obstacle pixel vector: " << endl;
    //         image2map(contours[contours_index[i]], contours_p);
    //         contours_map.push_back(contours_p);
    //         cout << "obstacle point in image: ";
    //         for (auto cont : contours[i]) {
    //             cout << "[" << cont.x << "," << cont.y <<"]" << " ";
    //         }
    //         cout << "\n";
    //         cout << "obstacle point in map: ";
    //         for (auto cont : contours_p) {
    //             cout << "[" << cont.x << "," << cont.y <<"]" << " ";
    //         }
    //         cout << "\n";
    //         double obs_area = area.at<float>(contours_index[i]) * m_image_height / image_height * m_image_width / image_width;
    //         cout << "obstacle area value in map:" << obs_area << endl;
    //     }

    //     // vector<Moments> mu(contours.size());
    //     // mu[max] = moments(contours[max], false);

    //     // vector<Point2f> mc(contours.size());
    //     // mc[max] = Point2d(mu[max].m10 / mu[max].m00, mu[max].m01 / mu[max].m00);

    //     // imshow("result image", imgcolor);
    //     // cout << "x= " << (int)mc[max].x << "****" << "y= " << (int)mc[max].y << endl;
    //     // imshow("Thresholded Image", ThresholdedImg);
    //     // cv::waitKey(3); 
    // }
}

void LocalMapUpdate::image2map(const vector<Point>& p, vector<Point2d>& p_map) {
    for (int i=0; i<p.size(); i++) {
        Point2d xy = pixel2coordinate(p[i]);
        p_map.push_back(xy);
    }
}

Point2d LocalMapUpdate::pixel2coordinate(Point p) {
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

void LocalMapUpdate::currentposCallback(const geometry_msgs::PoseStamped& odom1) {   
    posvector[0] = odom1.pose.position.x;
    posvector[1] = odom1.pose.position.y;
    posvector[2] = odom1.pose.position.z;
    posvector[3] = odom1.pose.orientation.x; 
    posvector[4] = odom1.pose.orientation.y;
    posvector[5] = odom1.pose.orientation.z;
    posvector[6] = odom1.pose.orientation.w;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localmap_update");
    string str1 = "/drone1/downward_cam/camera/image";
    string str2 = "/drone2/downward_cam/camera/image";
    string str3 = "/drone3/downward_cam/camera/image";
    string str4 = "/drone4/downward_cam/camera/image";
    ros::NodeHandle nh;
    LocalMapUpdate lm1(&nh, &nh, str1);
    ros::NodeHandle nh1;
    LocalMapUpdate lm2(&nh1, &nh1, str2);    
    ros::NodeHandle nh2;
    LocalMapUpdate lm3(&nh2, &nh2, str3);
    ros::NodeHandle nh3;
    LocalMapUpdate lm4(&nh3, &nh3, str4);

    ros::spin();
    return 0;
}