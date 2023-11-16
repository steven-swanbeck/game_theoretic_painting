#include "coverage_contest/game_visualizer.h"

GameVisualizer::GameVisualizer (ros::NodeHandle nh) : nh_(nh) {
    ROS_INFO_STREAM("[GameVisualizer] Created game visualizer.");

    // Create marker publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);

    ROS_INFO_STREAM("[GameVisualizer] Created a marker publisher.");

    // Create environment point cloud publisher
    env_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_topic_, 10);

    ROS_INFO_STREAM("[GameVisualizer] Created an environment point cloud publisher.");
}

void GameVisualizer::clearVisualizer(void) {
    // Clear all markers
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker);

    // Clear all points
    sensor_msgs::PointCloud2 points;    // New empty point cloud
    env_points_ = points;
    env_points_pub_.publish(env_points_); // Publish to environment
    for (const auto &p : players_) {
        p.second.points_pub.publish(points); // Publish to each player
    }

    // TODO Test that publisher is deleted for player.
    // Save publisher to variable, the clear, then try to publish.

    // Clear all players
    players_.clear();

    ROS_INFO_STREAM("[GameVisualizer] Cleared game visualizer.");
}

void GameVisualizer::addEnvironmentMarker(std::string mesh, std::vector<float_t> position) {
    // Set marker message
    env_marker_.header.frame_id = frame_id_;
    env_marker_.ns = "environment";
    env_marker_.id = 0;
    env_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    env_marker_.action = visualization_msgs::Marker::ADD;
    env_marker_.pose.position.x = position.at(0);
    env_marker_.pose.position.y = position.at(1);
    env_marker_.pose.position.z = position.at(2);
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

    ROS_INFO_STREAM("[GameVisualizer] Added an environment marker.");
}

void GameVisualizer::publishEnvironmentMarker(void) {
    // Adjust marker
    env_marker_.header.stamp = ros::Time::now();

    // Publish marker msg
    marker_pub_.publish(env_marker_);

    ROS_INFO_STREAM("[GameVisualizer] Published an environment marker.");
}

void GameVisualizer::addEnvironmentPoints(sensor_msgs::PointCloud2 points) {
    // Set points
    env_points_ = points;

    ROS_INFO_STREAM("[GameVisualizer] Added an environment point cloud.");
}

void GameVisualizer::publishEnvironmentPoints(void) {
    // Publish point cloud msg
    env_points_pub_.publish(env_points_);

    ROS_INFO_STREAM("[GameVisualizer] Published an environment point cloud.");
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

    // TODO Set point cloud color, if -1 randomize

    // Create point cloud publisher
    player_vis.points_pub = nh_.advertise<sensor_msgs::PointCloud2>(player_vis.id + points_topic_, 10);


    // Add to the player list
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

    // Convert and store back as point cloud msg
    pcl::toROSMsg(*pcl_points, player_vis->points);
}

void GameVisualizer::publishPlayerPoints(std::string player_id) {
    // Retrieve player visualizer
    PlayerVisualizer_t *player_vis = &players_.at(player_id);

    player_vis->points_pub.publish(player_vis->points);
}

void GameVisualizer::playTurn(std::string player_id, TurnSequence turn_sequence) {
    // TODO Steven

}
