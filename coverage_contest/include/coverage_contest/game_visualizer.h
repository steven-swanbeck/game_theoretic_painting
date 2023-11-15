#ifndef GAME_VISUALIZER_H
#define GAME_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "board.hpp"

// TODO
// #define ENVIRONMENT_MESH  ""
// #define DRONE_MESH        ""
// #define QUADRUPED_MESH    ""
// #define GANTRY_MESH       ""

struct PlayerVisualizer_t {
  std::string id;
  visualization_msgs::Marker marker;
  sensor_msgs::PointCloud2 points;
  std::vector<uint8_t> points_rgb{0, 0, 0};
  ros::Publisher points_pub;
};

class GameVisualizer {
  public:
    GameVisualizer (ros::NodeHandle &nh);

  private:
    void clearVisualizer(void);

    void addEnvironmentMarker(std::string mesh);

    void publishEnvironmentMarker(void);

    void addEnvironmentPoints(sensor_msgs::PointCloud2 points);

    void publishEnvironmentPoints(void);

    /** Adds a player to the game visualizer
     * @brief TODO
     * @param player_id string identifier for the player
     * @return TODO
     */
    void addPlayer(std::string player_id, std::string mesh);

    /** Move a player's marker
     * @brief TODO
     * @param player_id string identifier for the player
     * @param position cartesian plane position [x, y, z]
     * @return TODO
     */
    void movePlayerMarker(std::string player_id, std::vector<float_t> position);

    /** Publish a player's position to their respective topic
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void publishPlayerMarker(std::string player_id);

    /** Appends points to a player's point cloud
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void addPlayerPoints(std::string player_id, sensor_msgs::PointCloud2 points);

    /** Publish a player's cloud to their respective topic
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void publishPlayerPoints(std::string player_id);

    void displayPlayerTurn(std::string player_id, TurnSequence turn_sequence);

    // Private variables
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    std::string marker_topic_ = "/visualization_marker";
    visualization_msgs::Marker env_marker_;
    ros::Publisher env_points_pub_;
    std::string points_topic_ = "/points";
    sensor_msgs::PointCloud2 env_points_;
    std::map<std::string, PlayerVisualizer_t> players_;
    std::string frame_id_ = "map";
};


#endif // GAME_VISUALIZER_H
