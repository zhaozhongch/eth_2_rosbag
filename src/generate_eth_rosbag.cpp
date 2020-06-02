#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "cv_bridge/cv_bridge.h"

#include <fstream>
#include <iomanip>

int main(int argc, char** argv){
    ros::init(argc, argv, "eth_rosbag");

    ros::NodeHandle nh;

    ros::Publisher pub_imu       = nh.advertise<sensor_msgs::Imu>("/imu0", 100);
    ros::Publisher pub_gt        = nh.advertise<geometry_msgs::PoseStamped>("/groundtruth", 100);
    ros::Publisher pub_depth     = nh.advertise<cv_bridge::CvImage>("/depth", 100);
    ros::Publisher pub_rgb_right = nh.advertise<cv_bridge::CvImage>("/rgb_image0", 100);
    ros::Publisher pub_rgb_left  = nh.advertise<cv_bridge::CvImage>("/rgb_image1", 100);
    ros::Publisher pub_gray_right = nh.advertise<cv_bridge::CvImage>("/image0", 100);

    std::string dataset_address, dataset_name;
    if(nh.getParam("dataset_address", dataset_address)){
        ROS_INFO("we get the dataset_address %s", dataset_address.c_str());
    } 
    else{
        ROS_ERROR("no dataset_address, error");
    }

    if(nh.getParam("dataset_name", dataset_name)){
        ROS_INFO("we get the dataset_name %s", dataset_name.c_str());
    } 
    else{
        ROS_ERROR("no dataset_name, error");
    }

    bool skip_first_line_imu = true;
    bool skip_first_row_gt   = true;
    int sequence = 0;
    //imu frequency is about 2 times as groundtruth so I just publish 2 imu then publish one groundtruth
    double period_imu = 1.0/210.0; //imu frequency is about 210

    std::string depth_index_address, rgb_right_index_address, imu_address, groundtruth_address;
    std::string rgb_right_base_address, depth_base_address, rgb_left_base_address;
    imu_address             = dataset_address+dataset_name+"_imu/"+dataset_name+"/imu.txt";
    depth_index_address     = dataset_address+dataset_name+"_rgbd/"+dataset_name+"/depth.txt";
    rgb_right_index_address = dataset_address+dataset_name+"_mono/"+dataset_name+"/rgb.txt";
    groundtruth_address     = dataset_address+dataset_name+"_mono/"+dataset_name+"/groundtruth.txt";

    rgb_left_base_address   = dataset_address+dataset_name+"_stereo/"+dataset_name+"/";
    rgb_right_base_address  = dataset_address+dataset_name+"_mono/"+dataset_name+"/";
    depth_base_address      = dataset_address+dataset_name+"_rgbd/"+dataset_name+"/";

    std::list<sensor_msgs::Imu> all_imu;
    std::list<sensor_msgs::Imu>::iterator it_imu;
    std::list<geometry_msgs::PoseStamped> all_gt;
    std::list<geometry_msgs::PoseStamped>::iterator it_gt;

    ros::Time before_imu;

    //read all imu data
    double test_time_before = 0;
    std::ifstream read_imu(imu_address);
    if(!read_imu){
        std::cout<<"error! no IMU address"<<std::endl;
    }
    else{
        std::string one_row_imu;
        while(getline(read_imu, one_row_imu)){
            if(skip_first_line_imu) {skip_first_line_imu = false; continue;};
            std::istringstream tmp_one_row_imu(one_row_imu);
            sensor_msgs::Imu imu_data;
            std::string string_imu;
            //The following code can split one row of IMU by comma
            while(getline(tmp_one_row_imu, string_imu, ' ')){
                switch(sequence){
                    case 0:{
                        double msg_timestamp = (double)atof(string_imu.c_str());
                        msg_timestamp = 1e9 * msg_timestamp;
                        uint64_t int_time = uint64_t(msg_timestamp);
                        imu_data.header.stamp.sec  = int_time/1e9;
                        imu_data.header.stamp.nsec = int_time%1000000000;
                        sequence++;
                    }
                        break;
                    case 1:{
                        imu_data.angular_velocity.x = (double)atof(string_imu.c_str());
                        sequence++;                        
                    }
                        break;
                    case 2:{
                        imu_data.angular_velocity.y = (double)atof(string_imu.c_str());
                        sequence++;                        
                    }
                        break;
                    case 3:{
                        imu_data.angular_velocity.z = (double)atof(string_imu.c_str());
                        sequence++;                        
                    }
                        break;
                    case 4:{
                        imu_data.linear_acceleration.x = (double)atof(string_imu.c_str());
                        sequence++;                        
                    }
                        break;
                    case 5:{
                        imu_data.linear_acceleration.y = (double)atof(string_imu.c_str());
                        sequence++;                        
                    }
                        break;
                    case 6:{
                        imu_data.linear_acceleration.z = (double)atof(string_imu.c_str());
                        sequence++;                        
                    }
                        break;
                    default:
                        break;
                }
            }
            sequence = 0;
            all_imu.push_back(imu_data);
        }
        ROS_INFO("imu data size %ld", all_imu.size());
    }



    //read groundtruth
    std::ifstream read_groundtruth(groundtruth_address);
    if(!read_groundtruth){
        ROS_FATAL("cannot find the file that contains groundtruth");
        exit(0);
    }
    else{
        std::string one_row_gt;
        while(getline(read_groundtruth,one_row_gt)){
            if(skip_first_row_gt) {skip_first_row_gt = false; continue;}
            std::istringstream temp_one_row_gt(one_row_gt);
            geometry_msgs::PoseStamped groundtruth; 
            std::string string_gt;
            while(getline(temp_one_row_gt, string_gt, ' ')){
                switch(sequence){
                    case 0:{
                        double msg_timestamp = (double)atof(string_gt.c_str());
                        msg_timestamp = 1e9 * msg_timestamp;
                        uint64_t int_time = uint64_t(msg_timestamp);
                        groundtruth.header.stamp.sec  = int_time/1e9;
                        groundtruth.header.stamp.nsec = int_time%1000000000;
                        sequence++;
                    }
                        break;
                    case 1:{
                        //the case 1,2,3 is position x,y,z
                        groundtruth.pose.position.x = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 2:{
                        groundtruth.pose.position.y = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 3:{
                        groundtruth.pose.position.z = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 4:{
                        groundtruth.pose.orientation.x = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 5:{
                        groundtruth.pose.orientation.y = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 6:{
                        groundtruth.pose.orientation.z = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 7:{
                        groundtruth.pose.orientation.w = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    default:
                        break;
                }
            }
            sequence = 0;
            all_gt.push_back(groundtruth);
        }
        ROS_INFO("size of groundtruth data %ld", all_gt.size());   
    }

    //read all pictures
    std::ifstream read_right_rgb(rgb_right_index_address);
    std::ifstream read_depth(depth_index_address);
    cv_bridge::CvImage cv_ptr_depth, cv_ptr_rgb_right, cv_ptr_rgb_left, cv_ptr_gray_right;
    cv_ptr_depth.encoding     = "16UC1";
    cv_ptr_rgb_right.encoding = "8UC3";//8UC3
    cv_ptr_rgb_left.encoding  = "8UC3";//8UC3
    cv_ptr_gray_right.encoding = "8UC1";
    if(!read_right_rgb || !read_depth){
        ROS_FATAL("wrong path to find pictures");
        exit(0);        
    }
    else{
        std::string one_row_depth, one_row_right_rgb, one_row_left_rgb;
        while(getline(read_depth, one_row_depth) && getline(read_right_rgb, one_row_right_rgb) && ros::ok()){
            std::istringstream tmp_one_row_depth(one_row_depth);
            std::istringstream tmp_one_row_right_rgb(one_row_right_rgb);
            std::string string_depth, string_right_rgb, string_left_rgb;
            while(getline(tmp_one_row_depth, string_depth, ' ') && getline(tmp_one_row_right_rgb, string_right_rgb,' ')){
                switch(sequence){
                    case 0:{
                        double msg_timestamp = (double)atof(string_depth.c_str());
                        //std::cout<<"timestamp of image "<<std::setprecision(12)<<msg_timestamp<<std::endl;
                        msg_timestamp = 1e9 * msg_timestamp;
                        uint64_t int_time = uint64_t(msg_timestamp);
                        uint64_t sec  = int_time/1e9;
                        uint64_t nsec = int_time%1000000000;
                        cv_ptr_depth.header.stamp.sec      = sec;
                        cv_ptr_depth.header.stamp.nsec     = nsec;
                        cv_ptr_rgb_right.header.stamp.sec  = sec;
                        cv_ptr_rgb_right.header.stamp.nsec = nsec;
                        cv_ptr_rgb_left.header.stamp.sec   = sec;
                        cv_ptr_rgb_left.header.stamp.nsec  = nsec;
                        cv_ptr_gray_right.header.stamp.sec  = sec;
                        cv_ptr_gray_right.header.stamp.nsec = nsec;
                        sequence++;                        
                    }
                        break;
                    case 1:{
                        cv::Mat depth      = cv::imread(depth_base_address + string_depth, -1);
                        cv::Mat rgb_right  = cv::imread(rgb_right_base_address + string_right_rgb, CV_LOAD_IMAGE_COLOR);
                        cv::Mat gray_right = cv::imread(rgb_right_base_address + string_right_rgb, CV_LOAD_IMAGE_GRAYSCALE);

                        //replace rgb with rgb2 because the name of the folder containing stereo image is rgb2
                        std::string replace("rgb");
                        size_t pos =  string_right_rgb.find("rgb");
                        string_right_rgb.replace(pos, replace.length(), "rgb2");

                        cv::Mat rgb_left  = cv::imread(rgb_left_base_address + string_right_rgb, CV_LOAD_IMAGE_COLOR ); //still use `string_right_rgb` after replacing
                        cv_ptr_depth.image     =  depth.clone();
                        cv_ptr_rgb_right.image = rgb_right;
                        cv_ptr_rgb_left.image  = rgb_left;
                        cv_ptr_gray_right.image = gray_right;
                    }
                        break;
                    default:
                        break;
                }
            };
            sequence = 0;
            before_imu = ros::Time::now();
            int count = 0;
            //publish imu based on its cycle. publish image if imu time and ground truth time is bigger than image time

            for(;;){
                it_imu=all_imu.begin(); it_gt=all_gt.begin();
                if(it_imu->header.stamp.toSec() <  cv_ptr_depth.header.stamp.toSec() && ros::Time::now().toSec() - before_imu.toSec() >= period_imu){
                    count++;
                    before_imu = ros::Time::now();
                    pub_imu.publish(*it_imu);
                    all_imu.pop_front();
                    if(count == 2 && all_gt.size()>0){
                        pub_gt.publish(*it_gt);
                        all_gt.pop_front();
                        count = 0;
                    }
                }
                if(it_imu->header.stamp.toSec() >  cv_ptr_depth.header.stamp.toSec() || all_imu.size() == 0){
                    before_imu = ros::Time::now();
                    pub_depth.publish(cv_ptr_depth);
                    pub_rgb_right.publish(cv_ptr_rgb_right);
                    pub_rgb_left.publish(cv_ptr_rgb_left);
                    pub_gray_right.publish(cv_ptr_gray_right);
                    break;
                }
            }
            
        }
    }

    //ros::spin();
    std::cout<<"finish..............."<<std::endl;

    return 0;
}

    


