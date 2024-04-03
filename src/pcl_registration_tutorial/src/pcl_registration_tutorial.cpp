#include "pcl_registration_tutorial.hpp"


PclRegistrationTutorial::PclRegistrationTutorial()
{
    // Point cloud Pointer 초기화
    pcd_source_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_target_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

}

PclRegistrationTutorial::~PclRegistrationTutorial()
{

}

void PclRegistrationTutorial::Init()
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
    }

    ROS_INFO("Init Done");
    b_is_init_ = true;
}

bool PclRegistrationTutorial::CheckParam()
{
    ros::NodeHandle nh;
    if (!nh.getParam("/pcd_path/pcd_source", cfg_str_pcd_source_path_)) return false;  
    if (!nh.getParam("/pcd_path/pcd_target", cfg_str_pcd_target_path_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/registration_method", cfg_i_registration_method_)) return false;  

    std::string dir(ros::package::getPath("pcl_registration_tutorial") + "/../../");
    cfg_str_pcd_source_path_ = dir + cfg_str_pcd_source_path_;
    cfg_str_pcd_target_path_ = dir + cfg_str_pcd_target_path_;

    std::cout<<"cfg_str_pcd_source_path_: "<<cfg_str_pcd_source_path_<<std::endl;
    std::cout<<"cfg_str_pcd_target_path_: "<<cfg_str_pcd_target_path_<<std::endl;
    std::cout<<"cfg_i_registration_method_: "<<cfg_i_registration_method_<<std::endl;

    return true;
}

void PclRegistrationTutorial::Run()
{
    ROS_INFO("Run");
    if(b_is_init_ == false) return;
    
    switch(cfg_i_registration_method_){
        case EnumRegistrationMethod::ICP:
            IcpRegistration();
            break;

        case EnumRegistrationMethod::GICP:
            GicpRegistration();
            break;

        default:
            ROS_WARN("No Matching Method! %d", cfg_i_registration_method_);
            return;
            break;
    }

    UpdatePointCloud();

}

void PclRegistrationTutorial::IcpRegistration()
{
    ROS_INFO("Run ICP Started");
}


void PclRegistrationTutorial::GicpRegistration()
{
    ROS_INFO("Run GICP Started");
}

void PclRegistrationTutorial::UpdatePointCloud()
{
    ROS_INFO("UpdatePointCloud");

    pcl::toROSMsg(*pcd_source_pcptr_, o_pcd_source_origin_msg_);
    o_pcd_source_origin_msg_.header.stamp = ros::Time::now();
    o_pcd_source_origin_msg_.header.frame_id = "world";

    pcl::toROSMsg(*pcd_target_pcptr_, o_pcd_target_origin_msg_);
    o_pcd_target_origin_msg_.header.stamp = ros::Time::now();
    o_pcd_target_origin_msg_.header.frame_id = "world";
}

void PclRegistrationTutorial::Publish()
{
    ROS_INFO("Publish Data");
    if(b_is_init_ == false) return;

    p_pcd_source_origin_.publish(o_pcd_source_origin_msg_);
    p_pcd_target_origin_.publish(o_pcd_target_origin_msg_);
}



int main(int argc, char **argv) {
    std::string node_name = "pcl_registration_tutorial";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    PclRegistrationTutorial PRT;

    PRT.Init();
    PRT.Run();
    
    ros::Rate rate(1); // 1 Hz
    while (ros::ok()) {
        PRT.Publish();
        rate.sleep();    // Sleep to control the loop rate
    }

    return 0;
}