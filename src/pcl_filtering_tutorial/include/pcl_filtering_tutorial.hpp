// STD header
#include <iostream>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

// ROS header
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>


// Utility header
#include <boost/filesystem.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

class PclFilteringTutorial{
    public:
    PclFilteringTutorial();
    ~PclFilteringTutorial();

    void Init();
    void Run();
    void Publish();

    bool CheckParam();

    private: // ROS 송수신

    ros::Publisher p_pcd_source_origin_;
    ros::Publisher p_pcd_target_origin_;

    sensor_msgs::PointCloud2 o_pcd_source_origin_msg_;
    sensor_msgs::PointCloud2 o_pcd_target_origin_msg_;

    private: // 내부 자료

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_source_pcptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_target_pcptr_;

    private: // 설정값

    bool b_is_init_ = false;

    std::string cfg_str_pcd_source_path_;
    std::string cfg_str_pcd_target_path_;
    int cfg_i_registration_method_;


};