#include "coverage_contest/game_visualizer.h"

GameVisualizer::GameVisualizer (ros::NodeHandle nh) : nh_(nh) {
    ROS_INFO_STREAM("[GameVisualizer] Created game visualizer.");

    // Create marker publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
}

void GameVisualizer::clearVisualizer(void) {
    // Create an empty point cloud msg
    sensor_msgs::PointCloud2 points;
    PointCloud::Ptr pcl_points (new PointCloud);
    pcl::toROSMsg(*pcl_points, points);
    points.header.frame_id = frame_id_;
    
    // Clear all markers
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker);

    // Clear environment points
    EnvironmentVisualizer_t *env_vis;
    for (const auto &p : envs_) {
        env_vis = &envs_.at(p.first);
        env_vis->points_pub.publish(points);
    }

    // Clear player points
    PlayerVisualizer_t *player_vis;
    for (const auto &p : players_) {
        player_vis = &players_.at(p.first);
        player_vis->points_pub.publish(points);
    }

    ROS_INFO_STREAM("[GameVisualizer] Cleared game visualizer.");
}

void GameVisualizer::clearObjects(void) {
    // Clear environments map
    envs_.clear();

    // Clear players map
    players_.clear();
}

void GameVisualizer::addEnvironment(std::string env_id, std::vector<int16_t> points_rgb) {
    // Create new environment
    EnvironmentVisualizer_t env_vis;

    // Set environment id
    env_vis.id = env_id;

    // TODO Set point cloud color, if -1 randomize.
    env_vis.points_rgb.at(0) = points_rgb.at(0);
    env_vis.points_rgb.at(1) = points_rgb.at(1);
    env_vis.points_rgb.at(2) = points_rgb.at(2);

    // Add initial empty point cloud
    PointCloud::Ptr pcl_points (new PointCloud);
    pcl::toROSMsg(*pcl_points, env_vis.points);

    // Create point cloud publisher
    env_vis.points_pub = nh_.advertise<sensor_msgs::PointCloud2>(env_vis.id + points_topic_, 10);

    // Add to the environment map
    envs_.insert({env_vis.id, env_vis});

    ROS_INFO_STREAM("[GameVisualizer] Added an environment: " + env_id);
}

void GameVisualizer::addEnvironmentMarker(std::string env_id, std::string mesh, std::vector<float_t> position) {
    // Retrieve environment visualizer
    EnvironmentVisualizer_t *env_vis = &envs_.at(env_id);

    // Set marker message
    env_vis->marker.header.frame_id = frame_id_;
    env_vis->marker.ns = "environment";
    env_vis->marker.id = 0;
    env_vis->marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    env_vis->marker.action = visualization_msgs::Marker::ADD;
    env_vis->marker.pose.position.x = position.at(0);
    env_vis->marker.pose.position.y = position.at(1);
    env_vis->marker.pose.position.z = position.at(2);
    env_vis->marker.pose.orientation.x = 0.0;
    env_vis->marker.pose.orientation.y = 0.0;
    env_vis->marker.pose.orientation.z = 0.0;
    env_vis->marker.pose.orientation.w = 1.0;
    env_vis->marker.scale.x = 1.0;
    env_vis->marker.scale.y = 1.0;
    env_vis->marker.scale.z = 1.0;
    env_vis->marker.mesh_resource = mesh;
    env_vis->marker.mesh_use_embedded_materials = true;
    env_vis->marker.lifetime = ros::Duration();

    ROS_INFO_STREAM("[GameVisualizer] Added environment marker: " + env_id);
}

void GameVisualizer::publishEnvironmentMarker(std::string env_id) {
    // Retrieve environment visualizer
    EnvironmentVisualizer_t *env_vis = &envs_.at(env_id);

    // Adjust marker
    env_vis->marker.header.stamp = ros::Time::now();

    // Publish marker msg
    marker_pub_.publish(env_vis->marker);

    ROS_INFO_STREAM("[GameVisualizer] Published environment marker: " + env_id);
}

void GameVisualizer::addEnvironmentPoints(std::string env_id, std::string pcd) {
    // Retrieve environment visualizer
    EnvironmentVisualizer_t *env_vis = &envs_.at(env_id);

    // Read PCD file
    pcl::PCDReader reader;
    PointCloud::Ptr pcl_points (new PointCloud);
    reader.read(pcd, *pcl_points);

    // Color the points
    for (std::size_t i = 0; i < pcl_points->points.size(); i++) {
        pcl_points->points[i].r = env_vis->points_rgb.at(0);
        pcl_points->points[i].g = env_vis->points_rgb.at(1);
        pcl_points->points[i].b = env_vis->points_rgb.at(2);
    }

    // Set point cloud message
    pcl::toROSMsg(*pcl_points, env_vis->points);
    env_vis->points.header.frame_id = frame_id_;

    ROS_INFO_STREAM("[GameVisualizer] Added environment points: " + env_id);
}

void GameVisualizer::publishEnvironmentPoints(std::string env_id) {
    // Retrieve environment visualizer
    EnvironmentVisualizer_t *env_vis = &envs_.at(env_id);

    // Publish point cloud msg
    env_vis->points_pub.publish(env_vis->points);

    ROS_INFO_STREAM("[GameVisualizer] Published environment points: " + env_id);
}

void GameVisualizer::addPlayer(std::string player_id, std::vector<int16_t>points_rgb, std::string mesh) {
    // Create new player
    PlayerVisualizer_t player_vis;

    // Set player id
    player_vis.id = player_id;

    // Set marker
    player_vis.marker.header.frame_id = frame_id_;
    player_vis.marker.ns = player_vis.id;
    player_vis.marker.id = 0;
    player_vis.marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    player_vis.marker.action = visualization_msgs::Marker::ADD;
    player_vis.marker.pose.position.x = 30.0;
    player_vis.marker.pose.position.y = 0.0;
    player_vis.marker.pose.position.z = -13.0;
    player_vis.marker.pose.orientation.x = 0.0;
    player_vis.marker.pose.orientation.y = 0.0;
    player_vis.marker.pose.orientation.z = 0.0;
    player_vis.marker.pose.orientation.w = 1.0;
    player_vis.marker.scale.x = 1.0;
    player_vis.marker.scale.y = 1.0;
    player_vis.marker.scale.z = 1.0;
    player_vis.marker.mesh_resource = mesh;
    player_vis.marker.mesh_use_embedded_materials = true;
    player_vis.marker.lifetime = ros::Duration();

    // TODO Set point cloud color, if -1 randomize.
    player_vis.points_rgb.at(0) = points_rgb.at(0);
    player_vis.points_rgb.at(1) = points_rgb.at(1);
    player_vis.points_rgb.at(2) = points_rgb.at(2);

    // Add initial empty point cloud
    PointCloud::Ptr pcl_points (new PointCloud);
    pcl::toROSMsg(*pcl_points, player_vis.points);

    // Create point cloud publisher
    player_vis.points_pub = nh_.advertise<sensor_msgs::PointCloud2>(player_vis.id + points_topic_, 10);

    // Add to the player map
    players_.insert({player_vis.id, player_vis});

    ROS_INFO_STREAM("[GameVisualizer] Added a player: " + player_id);
}

void GameVisualizer::movePlayerMarker(std::string player_id, std::vector<float_t> position) {
    // Retrieve player visualizer
    PlayerVisualizer_t *player_vis = &players_.at(player_id);

    // Update marker positions
    player_vis->marker.pose.position.x = position.at(0);
    player_vis->marker.pose.position.y = position.at(1);
    player_vis->marker.pose.position.z = position.at(2);
}

void GameVisualizer::publishPlayerMarker(std::string player_id) {
    // Retrieve player visualizer
    PlayerVisualizer_t *player_vis = &players_.at(player_id);
    
    // Adjust marker
    player_vis->marker.header.stamp = ros::Time::now();

    // Publish marker msg
    marker_pub_.publish(player_vis->marker);
}

void GameVisualizer::addPlayerPoints(std::string player_id,  sensor_msgs::PointCloud2 new_points) {
    // Retrieve player visualizer
    PlayerVisualizer_t *player_vis = &players_.at(player_id);

    // Create point cloud pointers (with color parameters)
    PointCloud::Ptr pcl_points (new PointCloud);
    PointCloud::Ptr new_pcl_points (new PointCloud);

    // Convert from given point cloud msgs
    pcl::fromROSMsg(player_vis->points, *pcl_points);
    pcl::fromROSMsg(new_points, *new_pcl_points);

    // Color the new points
    for (std::size_t i = 0; i < new_pcl_points->points.size(); i++) {
        new_pcl_points->points[i].r = player_vis->points_rgb.at(0);
        new_pcl_points->points[i].g = player_vis->points_rgb.at(1);
        new_pcl_points->points[i].b = player_vis->points_rgb.at(2);
    }

    // Add new points
    *pcl_points += *new_pcl_points;

    // Convert and set point cloud msg
    pcl::toROSMsg(*pcl_points, player_vis->points);
    player_vis->points.header.frame_id = frame_id_;
}

void GameVisualizer::publishPlayerPoints(std::string player_id) {
    // Retrieve player visualizer
    PlayerVisualizer_t *player_vis = &players_.at(player_id);

    player_vis->points_pub.publish(player_vis->points);
}

void GameVisualizer::playTurn(std::string player_id, TurnSequence turn_sequence) {
    // TODO Steven
    // iterate through TurnSequence Actions (movement)
    //      movePlayerMarker()
    //      publishPlayerMarker()
    //      sleep(1)
    //      ...
    //      addPlayerPoints()
    //      publishPlayerPoints()
    //      sleep(1)
}
