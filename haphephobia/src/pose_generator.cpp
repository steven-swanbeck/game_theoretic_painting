#include "haphephobia/pose_generator.h"
#include "haphephobia/create_compliant_poses.h"

// . Constructor
CompliantPoseGenerationServer::CompliantPoseGenerationServer()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    nh_.getParam("/surface_repair/processing/virtual_fixtures/execution/vf_number", vf_number_);
    nh_.getParam("/robot/virtual_fixtures/tool_tip/offsets", tool_tip_offsets_);
    nh_.getParam("/robot/virtual_fixtures/tool_tip/calibration", tool_tip_calibration_);
    nh_.getParam("/robot/virtual_fixtures/tool_tip/reference_axis", reference_axis_);
    nh_.getParam("/robot/frames/root_link", robot_base_link_);
    nh_.getParam("/robot/frames/eef_link", robot_eef_link_);
    nh_.getParam("/robot/manipulation/planning_group", planning_group_);
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/vf_surface_offset", target_surface_offset_);

    ros::ServiceServer pose_generation_service_ = nh_.advertiseService("compliant_pose_generation", &CompliantPoseGenerationServer::generationCallback, this);
    ros::ServiceServer noncompliant_pose_generation_service_ = nh_.advertiseService("noncompliant_pose_generation", &CompliantPoseGenerationServer::noncompliantGenerationCallback, this);

    ros::waitForShutdown();
}

// . Primary callback
bool CompliantPoseGenerationServer::generationCallback (haphephobia::create_compliant_poses::Request &req, haphephobia::create_compliant_poses::Response &res)
{
    ros::AsyncSpinner spinner(1); // - only doing this because MoveIt stuff is generally quite unfriendly to a class structure so this is my stupid workaround
    spinner.start();

    // - Setup MoveIt stuffs
    moveit::planning_interface::MoveGroupInterface move_group_interface(planning_group_);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(planning_group_);
    move_group_interface.setPlanningTime(0.1);

    // - Preparing VF cloud
    // - transforming into robot body frame
    sensor_msgs::PointCloud2 vf_cloud_msg;
    Eigen::Matrix4f test {CompliantPoseGenerationServer::applyTransform(req.vf_cloud, vf_cloud_msg, robot_base_link_)};
    // - and converting to PCL
    pcl::PointCloud<pcl::PointNormal>::Ptr vf_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(vf_cloud_msg, *vf_cloud);

    // - Now create the initial candidate poses
    std::vector<geometry_msgs::Pose> initial_vf_pose_candidates(vf_cloud->points.size());
    std::vector<geometry_msgs::Pose> initial_eef_pose_candidates(vf_cloud->points.size());
    CompliantPoseGenerationServer::generateInitialPoses (vf_cloud, initial_vf_pose_candidates, initial_eef_pose_candidates);

    // - Now assess ability to execute each
    std::vector<bool> initial_planning_success (initial_eef_pose_candidates.size(), false);
    CompliantPoseGenerationServer::checkPoseFeasibility (move_group_interface, initial_eef_pose_candidates, initial_planning_success);

    // - For each that failed, generate compliant candidate clouds
    std::vector<pcl::PointCloud<pcl::PointNormal>> resampled_vf_clouds (initial_eef_pose_candidates.size());
    CompliantPoseGenerationServer::generateCompliantVFClouds (vf_cloud, initial_planning_success, resampled_vf_clouds);

    // // ! writing clouds to file for now
    // pcl::PCDWriter writer;
    // int cloud_num {};
    // for (std::size_t i = 0; i < initial_planning_success.size(); i++) {
    //     if (initial_planning_success[i]) {continue;}
    //     cloud_num++;
    //     writer.write("/home/steven/.ros/" + std::to_string(cloud_num) + ".pcd", resampled_vf_clouds[i], false);
    // }

    // - Now generate poses from the clouds
    std::vector<std::vector<geometry_msgs::Pose>> resampled_vf_poses (initial_vf_pose_candidates.size());
    std::vector<std::vector<geometry_msgs::Pose>> resampled_eef_poses (initial_vf_pose_candidates.size());
    CompliantPoseGenerationServer::generateResampledPoses(resampled_vf_clouds, initial_planning_success, resampled_vf_poses, resampled_eef_poses);

    // - Now reorder them according to some optimality criteria
    // ~ for now, this can probably be skipped
    // TODO add in this functionality

    // - Now try to plan to them, and add the first success to the final eef_pose list
    std::vector<bool> final_planning_success {initial_planning_success};
    std::vector<geometry_msgs::Pose> vf_poses (initial_vf_pose_candidates.size());
    std::vector<geometry_msgs::Pose> eef_poses (initial_vf_pose_candidates.size());
    for (std::size_t i = 0; i < initial_planning_success.size(); i++) {
        // - guard clause to make sure we do not replan what already succeeded
        if (initial_planning_success[i]) {
            vf_poses[i] = initial_vf_pose_candidates[i];    
            eef_poses[i] = initial_eef_pose_candidates[i];    
            continue;
        }

        PoseSuccess feasible_pose {CompliantPoseGenerationServer::findFeasiblePose(move_group_interface, resampled_eef_poses[i])};
        if (feasible_pose.planning_success) {
            vf_poses[i] = resampled_vf_poses[i][feasible_pose.original_pose_index]; // ! Note that, as constructed, this pose does not accurately relfect the rotation about the primary axis (not super useful RIGHT NOW because it is just used for visualization purposes, but might become important later)!!!
            eef_poses[i] = feasible_pose.vf_pose;
            final_planning_success[i] = true;
        } else {
            ROS_WARN_STREAM("No compliant resampled pose found! Defaulting to the originally planned pose, which WILL fail.");
            vf_poses[i] = initial_vf_pose_candidates[i];    
            eef_poses[i] = initial_eef_pose_candidates[i];
        }
    }

    // - Log info to user
    if (initial_planning_success.size() != final_planning_success.size()) {ROS_ERROR_STREAM("INITIAL AND FINAL PLANNING SUCCESS VECTORS HAVE DIFFERENT LENGTHS!");}
    int initial_success_counter {static_cast<int>(std::count_if(initial_planning_success.begin(), initial_planning_success.end(), [](int i) {return i == true;}))};
    int initial_percent_success {static_cast<int>(std::round(100 * static_cast<float>(initial_success_counter) / (initial_planning_success.size())))};
    int final_success_counter {static_cast<int>(std::count_if(final_planning_success.begin(), final_planning_success.end(), [](int i) {return i == true;}))};
    int final_percent_success {static_cast<int>(std::round(100 * static_cast<float>(final_success_counter) / (final_planning_success.size())))};
    ROS_INFO_STREAM("Initially planned to " << initial_percent_success << "% of poses (" << initial_success_counter << "/" << initial_planning_success.size() << ").");
    ROS_INFO_STREAM("Compliantly planned to " << final_percent_success << "% of poses (" << final_success_counter << "/" << final_planning_success.size() << ").");

    // - Return the new list of poses
    res.vf_poses = vf_poses;
    res.eef_poses = eef_poses;
    
    // - Creating a vector of std_msgs::Bool to pass back for visualization purposes
    std::vector<int> planning_status (final_planning_success.size());
    for (std::size_t i = 0; i < final_planning_success.size(); i++) {
        if (!final_planning_success[i]) {
            planning_status[i] = 0;
        } else if (initial_planning_success[i]) {
            planning_status[i] = 1;
        } else if (!initial_planning_success[i] && final_planning_success[i]) {
            planning_status[i] = 2;
        }
    }
    res.planning_status = planning_status;
    return true;
}



// . Noncompliant pose generation callback
bool CompliantPoseGenerationServer::noncompliantGenerationCallback (haphephobia::create_compliant_poses::Request &req, haphephobia::create_compliant_poses::Response &res)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // - Setup MoveIt stuffs
    moveit::planning_interface::MoveGroupInterface move_group_interface(planning_group_);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(planning_group_);
    move_group_interface.setPlanningTime(0.1);
    ROS_INFO_STREAM("Past MoveIt setup stuffs");

    // - Preparing VF cloud
    // - transforming into robot body frame
    sensor_msgs::PointCloud2 vf_cloud_msg;
    Eigen::Matrix4f test {CompliantPoseGenerationServer::applyTransform(req.vf_cloud, vf_cloud_msg, robot_base_link_)};
    // - and converting to PCL
    pcl::PointCloud<pcl::PointNormal>::Ptr vf_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(vf_cloud_msg, *vf_cloud);

    // - Now create the initial candidate poses
    std::vector<geometry_msgs::Pose> initial_vf_pose_candidates(vf_cloud->points.size());
    std::vector<geometry_msgs::Pose> initial_eef_pose_candidates(vf_cloud->points.size());
    CompliantPoseGenerationServer::generateInitialPoses (vf_cloud, initial_vf_pose_candidates, initial_eef_pose_candidates);

    // - Now assess ability to execute each
    std::vector<bool> initial_planning_success (initial_eef_pose_candidates.size(), false);
    CompliantPoseGenerationServer::checkPoseFeasibility (move_group_interface, initial_eef_pose_candidates, initial_planning_success);

    // - Log info to user
    int initial_success_counter {static_cast<int>(std::count_if(initial_planning_success.begin(), initial_planning_success.end(), [](int i) {return i == true;}))};
    int initial_percent_success {static_cast<int>(std::round(100 * static_cast<float>(initial_success_counter) / (initial_planning_success.size())))};
    ROS_INFO_STREAM("Noncompliantly planned to " << initial_percent_success << "% of poses (" << initial_success_counter << "/" << initial_planning_success.size() << ").");

    // - Return the new list of poses
    res.vf_poses = initial_vf_pose_candidates;
    res.eef_poses = initial_eef_pose_candidates;
    // > Creating a vector of std_msgs::Bool to pass back for visualization purposes
    std::vector<int> planning_status (initial_planning_success.size());
    for (std::size_t i = 0; i < initial_planning_success.size(); i++) {
        if (!initial_planning_success[i]) {
            planning_status[i] = 0;
        } else {
            planning_status[i] = 1;
        }
    }
    res.planning_status = planning_status;
    return true;
}

// . Version of above function that instead performs the roll compliance stuff
PoseSuccess CompliantPoseGenerationServer::findFeasiblePose (moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<geometry_msgs::Pose> &poses)
{
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/vf_roll_resolution", vf_roll_resolution_);

    PoseSuccess output;
    output.planning_success = false;

    bool success {};
    for (std::size_t i = 0; i < poses.size(); i++) {
        // - Instantiating compliant pose to be filled in and planned to later
        geometry_msgs::Pose compliant_pose;
        compliant_pose.position.x = poses[i].position.x;
        compliant_pose.position.y = poses[i].position.y;
        compliant_pose.position.z = poses[i].position.z;

        // > Get rotation matrix of initial pose
        Eigen::Quaternionf q;
        q.w() = poses[i].orientation.w;
        q.x() = poses[i].orientation.x;
        q.y() = poses[i].orientation.y;
        q.z() = poses[i].orientation.z;
        Eigen::Matrix3f R = q.toRotationMatrix();

        double angle_increment {360 / vf_roll_resolution_ * (M_PI / 180)};

        for (std::size_t j = 0; j < vf_roll_resolution_; j++) {
            // - Vary the appropriate Euler angle
            double compliant_yaw = j * angle_increment;

            // - Convert back into a quaternion, calculate composite transform, and assign to pose
            Eigen::Quaternionf q_roll = Eigen::AngleAxisf(compliant_yaw, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f R_roll = q_roll.toRotationMatrix();

            Eigen::Matrix3f R_compliant = R * R_roll;
            Eigen::Quaternionf q_compliant(R_compliant);
            compliant_pose.orientation.w = q_compliant.w();
            compliant_pose.orientation.x = q_compliant.x();
            compliant_pose.orientation.y = q_compliant.y();
            compliant_pose.orientation.z = q_compliant.z();

            // - Try to plan
            success = CompliantPoseGenerationServer::checkPoseFeasibility (move_group_interface, compliant_pose);
            if (success) {
                output.planning_success = true;
                output.original_pose_index = static_cast<int>(i);
                output.roll_modifier = compliant_yaw;
                output.vf_pose = compliant_pose;
                return output;
            }
        }
    }
    return output;
}

// . Function to generate other poses from compliant clouds
void CompliantPoseGenerationServer::generateResampledPoses (const std::vector<pcl::PointCloud<pcl::PointNormal>> &resampled_clouds, const std::vector<bool> &initial_successes, std::vector<std::vector<geometry_msgs::Pose>> &vf_poses, std::vector<std::vector<geometry_msgs::Pose>> &eef_poses)
{
    for (std::size_t i = 0; i < resampled_clouds.size(); i++) {
        // - guard clause to prevent generation of poses for ones we were previously able to plan to
        if (initial_successes[i]) {continue;}

        // - now calling the previously made function to generate poses from cloud
        pcl::PointCloud<pcl::PointNormal>::Ptr resampled_cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(resampled_clouds[i], *resampled_cloud);
        vf_poses[i].resize(resampled_clouds[i].points.size());
        eef_poses[i].resize(resampled_clouds[i].points.size());
        CompliantPoseGenerationServer::generateInitialPoses (resampled_cloud, vf_poses[i], eef_poses[i]);
    }
}

// . Generate compliant poses for ones that could not be planned to
void CompliantPoseGenerationServer::generateCompliantVFClouds (const pcl::PointCloud<pcl::PointNormal>::Ptr &original_cloud, const std::vector<bool> &succeeded, std::vector<pcl::PointCloud<pcl::PointNormal>> &sample_clouds)
{
    // - resize new cloud to fit in here
    pcl::PointCloud<pcl::PointNormal>::Ptr surface_cloud (new pcl::PointCloud<pcl::PointNormal>);
    surface_cloud->width = original_cloud->width;
    surface_cloud->height = original_cloud->height;
    surface_cloud->is_dense = true;
    surface_cloud->points.resize(surface_cloud->width * surface_cloud->height);

    // - deproject vf point using normal and offset distance
    for (std::size_t i = 0; i < original_cloud->points.size(); i++) {
        surface_cloud->points[i].normal_x = -1 * original_cloud->points[i].normal_x;
        surface_cloud->points[i].normal_y = -1 * original_cloud->points[i].normal_y;
        surface_cloud->points[i].normal_z = -1 * original_cloud->points[i].normal_z;
        surface_cloud->points[i].x = original_cloud->points[i].x + original_cloud->points[i].normal_x * target_surface_offset_;
        surface_cloud->points[i].y = original_cloud->points[i].y + original_cloud->points[i].normal_y * target_surface_offset_;
        surface_cloud->points[i].z = original_cloud->points[i].z + original_cloud->points[i].normal_z * target_surface_offset_;
    }

    // // ! writing to files for now
    // original_cloud->width = original_cloud->points.size();
    // original_cloud->height = 1;
    // pcl::PCDWriter writer;
    // writer.write("/home/steven/.ros/vfs.pcd", *original_cloud, false);

    // surface_cloud->width = surface_cloud->points.size();
    // surface_cloud->height = 1;
    // writer.write("/home/steven/.ros/surf.pcd", *surface_cloud, false);

    // - write the general structure of one of the conical compliant clouds with relaxed constraints
    pcl::PointCloud<pcl::PointNormal>::Ptr conical_cloud {CompliantPoseGenerationServer::generateConicalCloud()};

    // // ! writing conical cloud too
    // conical_cloud->width = conical_cloud->points.size();
    // conical_cloud->height = 1;
    // writer.write("/home/steven/.ros/cone.pcd", *conical_cloud, false);

    // - now transform it into the frame of the original VF
    for (std::size_t i = 0; i < succeeded.size(); i++) {
        // - making sure we only try to replace the failed poses; guard clause to prevent the rest of the function from being nested
        if (succeeded[i]) {continue;}

        // - transform the previously created conical cloud into the frame of the original fixture using the basis generation function
        Eigen::Matrix4d transform = CompliantPoseGenerationServer::findTransformationMatrix(surface_cloud->points[i]);
        pcl::PointCloud<pcl::PointNormal>::Ptr pre_transform (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr post_transform (new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud(*conical_cloud, *pre_transform);
        pcl::transformPointCloudWithNormals(*pre_transform, *post_transform, transform);
        pcl::copyPointCloud(*post_transform, sample_clouds[i]);
    }
}

// . Generate transformation matrix given a point normal
Eigen::Matrix4d CompliantPoseGenerationServer::findTransformationMatrix (const pcl::PointNormal point)
{
    Eigen::Vector3d first_basis (point.normal_x, point.normal_y, point.normal_z);
    Eigen::Matrix3d rotation = CompliantPoseGenerationServer::findOrthonormalBasis (first_basis);
    Eigen::Vector3d position (point.x, point.y, point.z);
    Eigen::Matrix4d transformation;
    // - adding rotation
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            transformation(i,j) = rotation(i,j);
        }
    }

    // - adding position
    transformation(0,3) = position(0);
    transformation(1,3) = position(1);
    transformation(2,3) = position(2);

    // - last row
    transformation(3,0) = 0;
    transformation(3,1) = 0;
    transformation(3,2) = 0;
    transformation(3,3) = 1;

    return transformation;
}

// . Find orthonormal basis given a unit vector 
Eigen::Matrix3d CompliantPoseGenerationServer::findOrthonormalBasis (Eigen::Vector3d unit_vector, const std::string major_axis)
{
    unit_vector.normalize();

    // - first find a unit vector perpendicular to the input one (ie. xTy == 0)
    Eigen::Vector3d perp_vector(0,0,0);
    for (int i = 0; i < unit_vector.size(); i++) {
        if (std::abs(unit_vector(i)) > 0.00) {
            int j {(i < unit_vector.size() - 1) ? i + 1 : 0};
            perp_vector(j) = unit_vector(i);
            perp_vector(i) = -1 * unit_vector(j);

            perp_vector.normalize();
            break;
        }
    }

    // - now create a third unit vector perpendicular to each by taking the cross product of the first two bases
    Eigen::Vector3d third_basis {unit_vector.cross(perp_vector)};

    // - now fill in the output rotation matrix using the original input unit vector as the specified major axis
    Eigen::Matrix3d rotation_matrix;
    if (major_axis == "x") {
        rotation_matrix(0,0) = unit_vector(0);
        rotation_matrix(1,0) = unit_vector(1);
        rotation_matrix(2,0) = unit_vector(2);
        rotation_matrix(0,1) = perp_vector(0);
        rotation_matrix(1,1) = perp_vector(1);
        rotation_matrix(2,1) = perp_vector(2);
        rotation_matrix(0,2) = third_basis(0);
        rotation_matrix(1,2) = third_basis(1);
        rotation_matrix(2,2) = third_basis(2);
    } else if (major_axis == "y") {
        rotation_matrix(0,1) = unit_vector(0);
        rotation_matrix(1,1) = unit_vector(1);
        rotation_matrix(2,1) = unit_vector(2);
        rotation_matrix(0,2) = perp_vector(0);
        rotation_matrix(1,2) = perp_vector(1);
        rotation_matrix(2,2) = perp_vector(2);
        rotation_matrix(0,0) = third_basis(0);
        rotation_matrix(1,0) = third_basis(1);
        rotation_matrix(2,0) = third_basis(2);
    } else {
        rotation_matrix(0,2) = unit_vector(0);
        rotation_matrix(1,2) = unit_vector(1);
        rotation_matrix(2,2) = unit_vector(2);
        rotation_matrix(0,0) = perp_vector(0);
        rotation_matrix(1,0) = perp_vector(1);
        rotation_matrix(2,0) = perp_vector(2);
        rotation_matrix(0,1) = third_basis(0);
        rotation_matrix(1,1) = third_basis(1);
        rotation_matrix(2,1) = third_basis(2);
    }

    return rotation_matrix;
}

// . Create conical cloud
pcl::PointCloud<pcl::PointNormal>::Ptr CompliantPoseGenerationServer::generateConicalCloud ()
{
    // - read in params related to how cloud should be created
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/min_vf_offset", min_vf_offset_);
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/max_vf_offset", max_vf_offset_);
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/vf_offset_resolution", vf_offset_resolution_);
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/vf_radial_resolution", vf_radial_resolution_);
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/max_vf_deviance_angle", max_vf_deviance_angle_);
    nh_.getParam("/surface_repair/processing/virtual_fixtures/generation/compliance/vf_deviance_resolution", vf_deviance_resolution_);

    pcl::PointCloud<pcl::PointNormal>::Ptr conical_cloud (new pcl::PointCloud<pcl::PointNormal>);
    
    // - first cycle through heights
    for (std::size_t i = 0; i < vf_offset_resolution_; i++) {
        // - calculate height at current iteration
        float offset_distance;

        // - this code will implictly assume that we want to get closer to the spot before trying to get further away
        float positive_offset_step_size {(max_vf_offset_ - target_surface_offset_) / std::floor(static_cast<float>((vf_offset_resolution_ - 1)) / 2)};
        float negative_offset_step_size {(min_vf_offset_ - target_surface_offset_) / std::ceil(static_cast<float>((vf_offset_resolution_ - 1)) / 2)};
        if (static_cast<int>(i) == 0) { // - set offset distance to desired on first iteration
            offset_distance = target_surface_offset_;
        } else {
            if ((static_cast<int>(i) - 1) % 2 == 0) { // - if (i - 1) is even, try a progressively closer offset distance
                offset_distance = target_surface_offset_ + negative_offset_step_size * std::ceil(static_cast<float>(static_cast<int>(i)) / 2); //? something that increments accordingly
            } else { // - else it is odd, try a progressively further distance
                offset_distance = target_surface_offset_ + positive_offset_step_size * std::ceil(static_cast<float>(static_cast<int>(i)) / 2);
            }
        }

        // - then through radial angles
        bool already_added_center_point {}; // - to avoid having several points all stacked up in the center
        for (std::size_t j = 0; j < vf_radial_resolution_; j++) {
            float radial {};
            float deviance {};
            // - calculate radial at current iteration
            if (vf_radial_resolution_ > 1) {
                // - adding in some offset to prevent structure from being too uniform
                radial = j * 360 / (vf_radial_resolution_) + (360 / (2 * vf_radial_resolution_)) * (static_cast<int>(std::ceil(static_cast<float>(static_cast<int>(i)) / 2)) % 2);

                // - then through deviance angles
                // - the deviance angle loop is inside the radial one because the deviance angle is meaningless if we have zero radial travel
                for (std::size_t k = 0; k < vf_deviance_resolution_; k++) {
                    // - calculate deviance at current iteration
                    if (vf_deviance_resolution_ > 1) {
                        deviance = k * max_vf_deviance_angle_ / (vf_deviance_resolution_ - 1);
                    }

                    if (deviance == 0) {
                        if (already_added_center_point) {
                            continue;
                        } else {
                            already_added_center_point = true;
                        }
                    }
                    
                    // - construct point for case when we have radial resolution
                    pcl::PointNormal cone_point;
                    cone_point.x = offset_distance * sin(deviance * M_PI / 180) * cos(radial * M_PI / 180);
                    cone_point.y = offset_distance * sin(deviance * M_PI / 180) * sin(radial * M_PI / 180);
                    cone_point.z = offset_distance * cos(deviance * M_PI / 180);

                    Eigen::Vector3d normal_vector(-1 * cone_point.x, -1 * cone_point.y, -1 * cone_point.z);
                    normal_vector.normalize();
                    cone_point.normal_x = normal_vector(0);
                    cone_point.normal_y = normal_vector(1);
                    cone_point.normal_z = normal_vector(2);

                    conical_cloud->points.push_back(cone_point);
                }

            } else { // - this is where we handle the case of having no radial resolution (ie. just moving in and out along the desired line of action)
                pcl::PointNormal cone_point;
                cone_point.x = 0;
                cone_point.y = 0;
                cone_point.z = offset_distance;
                cone_point.normal_x = 0;
                cone_point.normal_y = 0;
                cone_point.normal_z = -1;
                conical_cloud->points.push_back(cone_point);
            }
        }
    }
    conical_cloud->width = conical_cloud->points.size();
    conical_cloud->height = 1;
    return conical_cloud;
}

// . Function to check the feasibility of list of poses
void CompliantPoseGenerationServer::checkPoseFeasibility (moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::vector<geometry_msgs::Pose> &poses, std::vector<bool> &successes)
{
    int success_counter{};
    int fail_counter{};
    for (std::size_t i = 0; i < poses.size(); i++) {
        successes[i] = CompliantPoseGenerationServer::checkPoseFeasibility (move_group_interface, poses[i]);
        if (successes[i]) {
            success_counter++;
        } else {
            fail_counter++;
        }
    }
    int percent_success {static_cast<int>(std::round(100 * static_cast<float>(success_counter) / (success_counter + fail_counter)))};
    ROS_INFO_STREAM("Successfully planned to " << percent_success << "% of poses (" << success_counter << "/" << success_counter + fail_counter << ").");
}

// . Primary pose feasibility-checking function
bool CompliantPoseGenerationServer::checkPoseFeasibility (moveit::planning_interface::MoveGroupInterface &move_group_interface, const geometry_msgs::Pose &pose)
{
    move_group_interface.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    move_group_interface.plan(pose_plan);
    return (move_group_interface.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);
}

// . Function to generate first set of candidate poses
void CompliantPoseGenerationServer::generateInitialPoses (const pcl::PointCloud<pcl::PointNormal>::Ptr &vf_cloud, std::vector<geometry_msgs::Pose> &vf_poses, std::vector<geometry_msgs::Pose> &eef_poses)
{
    // - preparing reference axis for later calculation
    std::vector<double> double_reference_axis {static_cast<double>(reference_axis_[0]), static_cast<double>(reference_axis_[1]), static_cast<double>(reference_axis_[2])};
    Eigen::Vector3d reference_axis(double_reference_axis.data());

    // - looping through VF cloud and calculating a candidate pose
    for (std::size_t i = 0; i < vf_cloud->points.size(); i++) {
        // - first filling position fields
        vf_poses[i].position.x = vf_cloud->points[i].x;
        vf_poses[i].position.y = vf_cloud->points[i].y;
        vf_poses[i].position.z = vf_cloud->points[i].z;

        // - now filling orientation
        Eigen::Vector3d normal(vf_cloud->points[i].normal_x, vf_cloud->points[i].normal_y, vf_cloud->points[i].normal_z);
        Eigen::Quaterniond q;
        Eigen::Vector3d a = reference_axis.cross(normal);
        double w {reference_axis.norm() * normal.norm() + reference_axis.dot(normal)};
        q.w() = w;
        q.vec() = a;
        q.normalize();

        vf_poses[i].orientation.w = q.w();
        vf_poses[i].orientation.x = q.vec()[0];
        vf_poses[i].orientation.y = q.vec()[1];
        vf_poses[i].orientation.z = q.vec()[2];

        // - now filling in the EEF pose using a simple backward projection
        eef_poses[i].orientation.w = vf_poses[i].orientation.w;
        eef_poses[i].orientation.x = vf_poses[i].orientation.x;
        eef_poses[i].orientation.y = vf_poses[i].orientation.y;
        eef_poses[i].orientation.z = vf_poses[i].orientation.z;

        eef_poses[i].position.x = vf_poses[i].position.x - vf_cloud->points[i].normal_x * tool_tip_offsets_[0];
        eef_poses[i].position.y = vf_poses[i].position.y - vf_cloud->points[i].normal_y * tool_tip_offsets_[0];
        eef_poses[i].position.z = vf_poses[i].position.z - vf_cloud->points[i].normal_z * tool_tip_offsets_[0];
    }   
}

// . Function to apply transforms to clouds as cloud msgs
Eigen::Matrix4f CompliantPoseGenerationServer::applyTransform (const sensor_msgs::PointCloud2 &cloud_in, sensor_msgs::PointCloud2 &cloud_out, std::string output_frame)
{
    Eigen::Matrix4f eigen_transform;
    if (cloud_in.header.frame_id == output_frame) {
        cloud_out = cloud_in;
        return eigen_transform;
    }
    tf::StampedTransform transform;
    try {
        listener_.lookupTransform(output_frame, cloud_in.header.frame_id, cloud_in.header.stamp, transform);
    }    
    catch (tf::LookupException &e) {
        ROS_ERROR("%s", e.what());
        return eigen_transform;
    }
    catch (tf::ExtrapolationException &e) {
        ROS_ERROR("%s", e.what());
        return eigen_transform;
    }
    pcl_ros::transformAsMatrix(transform, eigen_transform);

    pcl::PointCloud<pcl::PointNormal>::Ptr input (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(cloud_in, *input);

    pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*input, *output, eigen_transform);
    pcl::toROSMsg(*output, cloud_out);

    cloud_out.header.frame_id = output_frame;
    return eigen_transform;
}

int main (int argc, char** argv) 
{
    ros::init(argc, argv, "pose_generation_server");
    CompliantPoseGenerationServer compliant_pose_generation_server;

    return 0;
}
