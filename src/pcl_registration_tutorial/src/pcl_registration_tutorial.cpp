#include "pcl_registration_tutorial.hpp"


PclRegistrationTutorial::PclRegistrationTutorial()
{
    // Point cloud Pointer 초기화
    pcd_source_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_target_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_source_registered_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

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

    p_pcd_source_registered_ = nh.advertise<sensor_msgs::PointCloud2>("source_pcd_registered", 10);


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

    Eigen::Affine3f source_tranform_tf = pcl::getTransformation(cfg_d_target_move_x_m_, cfg_d_target_move_y_m_,cfg_d_target_move_z_m_,
            cfg_d_target_rot_roll_deg_*M_PI/180.0, cfg_d_target_rot_pitch_deg_*M_PI/180.0, cfg_d_target_rot_yaw_deg_*M_PI/180.0);

    pcl::transformPointCloud(*pcd_source_pcptr_, *pcd_source_pcptr_, source_tranform_tf);

    ROS_INFO("Init Done");
    b_is_init_ = true;
}

bool PclRegistrationTutorial::CheckParam()
{
    ros::NodeHandle nh;
    if (!nh.getParam("/pcd_path/pcd_source", cfg_str_pcd_source_path_)) return false;  
    if (!nh.getParam("/pcd_path/pcd_target", cfg_str_pcd_target_path_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/registration_method", cfg_i_registration_method_)) return false;  

    if (!nh.getParam("/pcl_registration_tutorial/target_move_x_m", cfg_d_target_move_x_m_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/target_move_y_m", cfg_d_target_move_y_m_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/target_move_z_m", cfg_d_target_move_z_m_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/target_rot_roll_deg", cfg_d_target_rot_roll_deg_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/target_rot_pitch_deg", cfg_d_target_rot_pitch_deg_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/target_rot_yaw_deg", cfg_d_target_rot_yaw_deg_)) return false;  
    if (!nh.getParam("/pcl_registration_tutorial/max_search_distance", cfg_d_max_search_distance_)) return false;  


    std::string dir(ros::package::getPath("pcl_registration_tutorial") + "/../../");
    cfg_str_pcd_source_path_ = dir + cfg_str_pcd_source_path_;
    cfg_str_pcd_target_path_ = dir + cfg_str_pcd_target_path_;

    std::cout<<"cfg_str_pcd_source_path_: "<<cfg_str_pcd_source_path_<<std::endl;
    std::cout<<"cfg_str_pcd_target_path_: "<<cfg_str_pcd_target_path_<<std::endl;
    std::cout<<"cfg_i_registration_method_: "<<cfg_i_registration_method_<<std::endl;

    std::cout<<"cfg_d_target_move_x_m_: "<<cfg_d_target_move_x_m_<<std::endl;
    std::cout<<"cfg_d_target_move_y_m_: "<<cfg_d_target_move_y_m_<<std::endl;
    std::cout<<"cfg_d_target_move_z_m_: "<<cfg_d_target_move_z_m_<<std::endl;
    std::cout<<"cfg_d_target_rot_roll_deg_: "<<cfg_d_target_rot_roll_deg_<<std::endl;
    std::cout<<"cfg_d_target_rot_pitch_deg_: "<<cfg_d_target_rot_pitch_deg_<<std::endl;
    std::cout<<"cfg_d_target_rot_yaw_deg_: "<<cfg_d_target_rot_yaw_deg_<<std::endl;

    return true;
}

void PclRegistrationTutorial::Run()
{
    
    if(b_is_init_ == false) return;
    ROS_INFO("Run");
    
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

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(cfg_d_max_search_distance_);
    icp.setTransformationEpsilon(0.003);
    icp.setMaximumIterations(1000);

    icp.setInputSource(pcd_source_pcptr_);
    icp.setInputTarget(pcd_target_pcptr_);
    icp.align(*pcd_source_registered_pcptr_);


}


void PclRegistrationTutorial::GicpRegistration()
{
    ROS_INFO("Run GICP Started");

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(cfg_d_max_search_distance_);
    gicp.setTransformationEpsilon(0.003);
    gicp.setMaximumIterations(1000);

    gicp.setInputSource(pcd_source_pcptr_);
    gicp.setInputTarget(pcd_target_pcptr_);
    gicp.align(*pcd_source_registered_pcptr_);

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

    pcl::toROSMsg(*pcd_source_registered_pcptr_, o_pcd_source_registered_msg_);
    o_pcd_source_registered_msg_.header.stamp = ros::Time::now();
    o_pcd_source_registered_msg_.header.frame_id = "world";

}

void PclRegistrationTutorial::Publish()
{
    if(b_is_init_ == false) return;
    ROS_INFO("Publish Data");

    p_pcd_source_origin_.publish(o_pcd_source_origin_msg_);
    p_pcd_target_origin_.publish(o_pcd_target_origin_msg_);

    p_pcd_source_registered_.publish(o_pcd_source_registered_msg_);
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