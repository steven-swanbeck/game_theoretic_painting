#ifndef GAME_VISUALIZER_H
#define GAME_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "board.hpp"
#include "point.h"


// TODO These two can be combined into the same object
struct EnvironmentVisualizer_t {
  std::string id;
  visualization_msgs::Marker marker;
  sensor_msgs::PointCloud2 points;
  std::vector<uint8_t> points_rgb{0, 0, 0};
  ros::Publisher points_pub;
};

struct PlayerVisualizer_t {
  std::string id;
  visualization_msgs::Marker marker;
  sensor_msgs::PointCloud2 points;
  std::vector<int16_t> points_rgb{0, 0, 0};
  ros::Publisher points_pub;
};

class GameVisualizer {
  public:
    GameVisualizer (ros::NodeHandle nh);

    void clearVisualizer(void);

    void clearObjects(void);

    void addEnvironment(std::string env_id, std::vector<int16_t>points_rgb);

    void addEnvironmentMarker(std::string env_id, std::string mesh, std::vector<float_t> position);

    void publishEnvironmentMarker(std::string env_id);

    void addEnvironmentPoints(std::string env_id, std::string pcd);

    void publishEnvironmentPoints(std::string env_id);

    /** Adds a player to the game visualizer
     * @brief TODO
     * @param player_id string identifier for the player
     * @return TODO
     */
    void addPlayer(std::string player_id, std::vector<int16_t>points_rgb, std::string mesh);

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

    void playTurn(std::string player_id, TurnSequence turn_sequence);

  private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;

    std::string marker_topic_ = "/visualization_marker";
    std::string points_topic_ = "/points";
    std::string frame_id_ = "map";

    std::map<std::string, EnvironmentVisualizer_t> envs_;
    std::map<std::string, PlayerVisualizer_t> players_;
};


#endif // GAME_VISUALIZER_H
