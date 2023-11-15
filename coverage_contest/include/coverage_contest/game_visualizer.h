#ifndef GAME_VISUALIZER_H
#define GAME_VISUALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

struct PlayerVisualizer {
  std::string id;
  sensor_msgs::PointCloud2 points;
  visualization_msgs::Marker marker;
};

class GameVisualizer {
  public:
    GameVisualizer (ros::NodeHandle &nh);

  private:
    /** Adds a player to the game visualizer
     * @brief TODO
     * @param player_id string identifier for the player
     * @return TODO
     */
    void addPlayer(std::string player_id, int type);

    /** Appends points to a player's point cloud
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void addPoints(std::string player_id, sensor_msgs::PointCloud2 cloud);

    /** Publish a player's cloud to their respective topic
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void publishPoints(std::string player_id);

    void GameVisualizer::movePlayer(std::string player_id, float[] pose);

    /** Publish a player's position to their respective topic
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void publishMarkers(std::string player_id);

    // Private variables
    ros::NodeHandle nh_;
    std::string points_topic_ = "/painted_points";
    ros::Publisher points_pub_;
    std::string marker_topic_ = "/visualization_marker";
    ros::Publisher marker_pub_;
    std::vector<PlayerVisualizer> players_;
    std::string frame_id_ = "map";
};


#endif // GAME_VISUALIZER_H
