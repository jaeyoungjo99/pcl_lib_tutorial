#include "pcl_filtering_tutorial.hpp"


PclFilteringTutorial::PclFilteringTutorial()
{
    // Point cloud Pointer 초기화
    pcd_source_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_target_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

}

PclFilteringTutorial::~PclFilteringTutorial()
{

}

void PclFilteringTutorial::Init()
{
    ROS_INFO("Init");

    ros::NodeHandle nh;

    if(CheckParam() == false){
        ROS_ERROR("Init Fail");
        return;
    }


    p_pcd_source_origin_ = nh.advertise<sensor_msgs::PointCloud2>("source_pcd_origin", 10);
    p_pcd_target_origin_ = nh.advertise<sensor_msgs::PointCloud2>("target_pcd_origin", 10);


    // Load PCD
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cfg_str_pcd_source_path_, *pcd_source_pcptr_) == -1){
        ROS_ERROR_STREAM("Cannot Read file: " << cfg_str_pcd_source_path_);
        return;
    }
    else{
        ROS_WARN_STREAM("File Loaded From: " << cfg_str_pcd_source_path_);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cfg_str_pcd_target_path_,*pcd_target_pcptr_) == -1){
        ROS_ERROR_STREAM("Cannot Read file: " << cfg_str_pcd_target_path_);
        return;
    }
    else{
        ROS_WARN_STREAM("File Loaded From: " << cfg_str_pcd_target_path_);
        return;
    }

    ROS_INFO("Init Done");
    b_is_init_ = true;
}

bool PclFilteringTutorial::CheckParam()
{
    ros::NodeHandle nh;
    if (!nh.getParam("/pcd_path/pcd_source", cfg_str_pcd_source_path_)) return false;  
    if (!nh.getParam("/pcd_path/pcd_target", cfg_str_pcd_target_path_)) return false;  
    if (!nh.getParam("/pcl_filtering_tutorial/registration_method", cfg_i_registration_method_)) return false;  

    std::string dir(ros::package::getPath("pcl_filtering_tutorial") + "/../../");
    cfg_str_pcd_source_path_ = dir + cfg_str_pcd_source_path_;
    cfg_str_pcd_target_path_ = dir + cfg_str_pcd_target_path_;

    std::cout<<"cfg_str_pcd_source_path_: "<<cfg_str_pcd_source_path_<<std::endl;
    std::cout<<"cfg_str_pcd_target_path_: "<<cfg_str_pcd_target_path_<<std::endl;
    std::cout<<"cfg_i_registration_method_: "<<cfg_i_registration_method_<<std::endl;

    return true;
}

void PclFilteringTutorial::Run()
{
    ROS_INFO("Run");
    if(b_is_init_ == false) return;

}

void PclFilteringTutorial::Publish()
{
    ROS_INFO("Publish Data");
    if(b_is_init_ == false) return;

    p_pcd_source_origin_.publish(o_pcd_source_origin_msg_);
    p_pcd_target_origin_.publish(o_pcd_target_origin_msg_);
}



int main(int argc, char **argv) {
    std::string node_name = "pcl_filtering_tutorial";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    PclFilteringTutorial PRT;

    PRT.Init();
    PRT.Run();
    
    ros::Rate rate(1); // 1 Hz
    while (ros::ok()) {
        PRT.Publish();
        rate.sleep();    // Sleep to control the loop rate
    }

    return 0;
}