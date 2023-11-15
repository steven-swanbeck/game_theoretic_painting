#include "coverage_contest/game_visualizer.h"

GameVisualizer::GameVisualizer (ros::NodeHandle &nh) {
    // TODO

    // Initialize node handle
    nh_ = nh;

    // Initialize point cloud publisher
    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_topic_, 1);

    // Initialize marker publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1);

    std::cout << "AAAAAAAA" << std::endl;
}

add GameVisualizer::addEnvironment() {

}

void GameVisualizer::addPlayer(std::string player_id, int type) {
    // TOD
    PlayerVisualizer player;

    switch(type) {
        case 0:     // drone
            break;
        case 1:     // quadruped
            break;
        case 2:
            break;  // gantry
    }

    // Construct marker object message
    player.marker.header.frame_id = frame_id_;
    player.marker.ns = player_id;
    player.marker.id = 0;
    player.marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    player.marker.action = visualization_msgs::Marker::ADD;
    player.marker.scale.x = 1.0;
    player.marker.scale.y = 1.0;
    player.marker.scale.z = 1.0;
    player.marker.mesh_resource = mesh;

}

void GameVisualizer::movePlayer(std::string player_id, float[] pose) {
    // TODO
}

void GameVisualizer::addPoints(std::string player_id,  sensor_msgs::PointCloud2 cloud) {
    // TODO
}

void GameVisualizer::publishPoints(std::string player_id) {
    // TODO
}

void GameVisualizer::publishMarkers(std::string player_id) {
    // TODO

    // player.marker.header.stamp = ros::Time::now();
}
