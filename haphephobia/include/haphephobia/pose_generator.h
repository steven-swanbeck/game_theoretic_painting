#ifndef VIRTUAL_FIXTURE_COMPLIANT_GENERATION_SERVER
#define VIRTUAL_FIXTURE_COMPLIANT_GENERATION_SERVER

#include "common.h"
#include "point.h"
#include "haphephobia/create_compliant_poses.h"
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <cmath>

struct VF
{
    sensor_msgs::PointCloud2 cluster;
    sensor_msgs::PointCloud2 vf_cloud;
    geometry_msgs::Pose base_pose;
    geometry_msgs::Pose camera_pose;
};

struct PoseSuccess
{
    bool planning_success;
    int original_pose_index;
    float roll_modifier;
    geometry_msgs::Pose vf_pose;
};

class CompliantPoseGenerationServer
{
public:
    CompliantPoseGenerationServer();
private:
    bool generationCallback(haphephobia::create_compliant_poses::Request &req, haphephobia::create_compliant_poses::Response &res);
    bool noncompliantGenerationCallback(haphephobia::create_compliant_poses::Request &req, haphephobia::create_compliant_poses::Response &res);
    Eigen::Matrix4f applyTransform (const sensor_msgs::PointCloud2 &cloud_in, sensor_msgs::PointCloud2 &cloud_out, std::string output_frame);
    void generateInitialPoses (const pcl::PointCloud<pcl::PointNormal>::Ptr &vf_cloud, std::vector<geometry_msgs::Pose> &vf_poses, std::vector<geometry_msgs::Pose> &eef_poses);
    void checkPoseFeasibility (moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<geometry_msgs::Pose> &poses, std::vector<bool> &successes);
    bool checkPoseFeasibility (moveit::planning_interface::MoveGroupInterface &move_group_interface, const geometry_msgs::Pose &pose);
    void generateCompliantVFClouds (const pcl::PointCloud<pcl::PointNormal>::Ptr &original_cloud, const std::vector<bool> &succeeded, std::vector<pcl::PointCloud<pcl::PointNormal>> &sample_clouds);
    Eigen::Matrix3d findOrthonormalBasis (Eigen::Vector3d unit_vector, const std::string major_axis="z");
    pcl::PointCloud<pcl::PointNormal>::Ptr generateConicalCloud ();
    Eigen::Matrix4d findTransformationMatrix (const pcl::PointNormal point);
    void generateResampledPoses (const std::vector<pcl::PointCloud<pcl::PointNormal>> &resampled_clouds, const std::vector<bool> &initial_successes, std::vector<std::vector<geometry_msgs::Pose>> &vf_poses, std::vector<std::vector<geometry_msgs::Pose>> &eef_poses);
    PoseSuccess findFeasiblePose (moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<geometry_msgs::Pose> &poses);

    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    ros::ServiceServer pose_generation_service_;
    ros::ServiceServer noncompliant_pose_generation_service_;

    int vf_number_;
    std::vector<float> tool_tip_offsets_;
    std::vector<double> tool_tip_calibration_;
    std::vector<float> reference_axis_;
    std::string robot_base_link_;
    std::string robot_eef_link_;
    std::string planning_group_;
    float target_surface_offset_;

    // - compliance params
    float min_vf_offset_;
    float max_vf_offset_;
    int vf_offset_resolution_;
    int vf_radial_resolution_;
    float max_vf_deviance_angle_;
    int vf_deviance_resolution_;
    int vf_roll_resolution_;
};

#endif //VIRTUAL_FIXTURE_COMPLIANT_GENERATION_SERVER
