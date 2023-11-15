#include "coverage_contest/game_visualizer.h"

GameVisualizer::GameVisualizer (ros::NodeHandle &nh) {
    // TODO Fix getting the node handle
    // Initialize node handle
    nh_ = nh;

    // Create marker publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
}

void GameVisualizer::clearVisualizer(void) {
    // Clear all markers
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker);

    // Clear all point clouds
    // TODO Clear environment
    // TODO Clear players'

    // Clear all players
    players_.clear();
}

void GameVisualizer::addEnvironmentMarker(std::string mesh) {
    // Set marker message
    env_marker_.header.frame_id = frame_id_;
    env_marker_.ns = "environment";
    env_marker_.id = 0;
    env_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    env_marker_.action = visualization_msgs::Marker::ADD;
    env_marker_.pose.position.x = 0.0;
    env_marker_.pose.position.y = 0.0;
    env_marker_.pose.position.z = 0.0;
    env_marker_.pose.orientation.x = 0.0;
    env_marker_.pose.orientation.y = 0.0;
    env_marker_.pose.orientation.z = 0.0;
    env_marker_.pose.orientation.w = 1.0;
    env_marker_.scale.x = 1.0;
    env_marker_.scale.y = 1.0;
    env_marker_.scale.z = 1.0;
    env_marker_.mesh_resource = mesh;
    env_marker_.mesh_use_embedded_materials = true;
    env_marker_.lifetime = ros::Duration();
}

void GameVisualizer::publishEnvironmentMarker(void) {
    env_points_pub_.publish(env_marker_);
}

void GameVisualizer::addEnvironmentPoints(sensor_msgs::PointCloud2 points) {
    // Create point cloud publisher
    env_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_topic_, 10);

    // Set points
    env_points_ = points;
}

void GameVisualizer::publishEnvironmentPoints(void) {
    env_points_pub_.publish(env_points_);
}

void GameVisualizer::addPlayer(std::string player_id, std::string mesh) {
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
    player_vis.marker.scale.x = 1.0;
    player_vis.marker.scale.y = 1.0;
    player_vis.marker.scale.z = 1.0;
    player_vis.marker.mesh_resource = mesh;
    player_vis.marker.mesh_use_embedded_materials = true;
    player_vis.marker.lifetime = ros::Duration();

    // Set point cloud color

    // Create point cloud publisher
    player_vis.points_pub = nh_.advertise<sensor_msgs::PointCloud2>(player_vis.id + points_topic_, 10);


    // Add to the player list
    // TODO Need to deepcopy?
    players_.insert({player_vis.id, player_vis});
}

void GameVisualizer::movePlayerMarker(std::string player_id, std::vector<float_t> position) {
    // Retrieve player visualizer
    PlayerVisualizer_t player_vis = players_.at(player_id);

    // Update marker positions
    player_vis.marker.pose.position.x = position.at(0);
    player_vis.marker.pose.position.x = position.at(1);
    player_vis.marker.pose.position.x = position.at(2);
}

void GameVisualizer::publishPlayerMarker(std::string player_id) {
    // Retrieve player visualizer
    PlayerVisualizer_t player_vis = players_.at(player_id);
    
    // Adjust marker
    player_vis.marker.header.stamp = ros::Time::now();

    // Publishe marker msg
    marker_pub_.publish(player_vis.marker);
}

void GameVisualizer::addPlayerPoints(std::string player_id,  sensor_msgs::PointCloud2 new_points) {
    pcl::PCLPointCloud2 pcl_points;
    pcl::PCLPointCloud2 new_pcl_points;

    // Retrieve player visualizer
    PlayerVisualizer_t player_vis = players_.at(player_id);

    // Convert to PCL points
    pcl_conversions::toPCL(player_vis.points, pcl_points);
    pcl_conversions::toPCL(new_points, new_pcl_points);

    // Add new points
    pcl::concatenate(pcl_points, new_pcl_points, pcl_points);

    // TODO Color points

    // Store back as msg
    pcl_conversions::fromPCL(pcl_points, player_vis.points);
}

void GameVisualizer::publishPlayerPoints(std::string player_id) {
    // Retrieve player visualizer
    PlayerVisualizer_t player_vis = players_.at(player_id);

    player_vis.points_pub.publish(player_vis.points);
}

void GameVisualizer::displayPlayerTurn(std::string player_id, TurnSequence turn_sequence) {
    // TODO
}
