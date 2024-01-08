#ifndef GAME_VISUALIZER_H
#define GAME_VISUALIZER_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include "board.hpp"
// #include "point.h"

struct Visualizer_t {
  std::string id;
  visualization_msgs::msg::Marker marker;
  sensor_msgs::msg::PointCloud2 points;
  std::vector<int16_t> points_rgb{0, 0, 0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
};

class GameVisualizer {
  public:
    GameVisualizer(rclcpp::Node::SharedPtr node);

    void clearVisualizer(void);

    void clearObjects(void);

    void addEnvironment(std::string env_id, std::vector<int16_t>points_rgb);

    void addEnvironmentMarker(std::string env_id, std::string mesh, std::vector<float_t> position, float alpha=1.0);

    void publishEnvironmentMarker(std::string env_id);

    void addEnvironmentPoints(std::string env_id, std::string pcd);
    
    void addEnvironmentPoints(std::string env_id, sensor_msgs::msg::PointCloud2 &cloud);
    
    // void addEnvironmentPoints(std::string env_id, PointCloud::Ptr &pcl_points);

    void publishEnvironmentPoints(std::string env_id);

    std::vector<int16_t> inputColors();

    /** Adds a player to the game visualizer
     * @brief TODO
     * @param player_id string identifier for the player
     * @return TODO
     */
    // void addPlayer(std::string player_id, std::vector<int16_t>points_rgb, std::string mesh);
    void addPlayer(std::string player_id, std::string mesh, bool input_color=false, bool use_color_for_player=true);

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
    void addPlayerPoints(std::string player_id, sensor_msgs::msg::PointCloud2 points);

    /** Publish a player's cloud to their respective topic
     * @brief TODO
     * @param player_id string identifier for the player
     * @param cloud point cloud to be appended
     * @return TODO
     */
    void publishPlayerPoints(std::string player_id);

  private:
    rclcpp::Node::SharedPtr node_;

    std::string marker_topic_ = "/marker";
    std::string points_topic_ = "/points";
    std::string frame_id_ = "map";

    std::map<std::string, Visualizer_t> envs_;
    std::map<std::string, Visualizer_t> players_;
};

#endif // GAME_VISUALIZER_H
