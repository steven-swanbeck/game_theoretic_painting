#include "coverage_contest/play_game.h"

GamePlayer::GamePlayer ()
{
    // . Loading data
    board_utils::loadCloudasMsg("/home/steven/game_theoretic_painting/src/models/clouds/revised/map.pcd", map_);
    board_utils::loadCloudasMsg("/home/steven/game_theoretic_painting/src/models/clouds/revised/marked.pcd", marked_);
    float map_discretization {3.0}; // 3 looks pretty good for the map we have
    
    // . Making board
    board_ = board_utils::generateBoard(map_, map_discretization);

    // . Instantiating players
    players_ = agents::instantiatePlayers(1, 1, 1);

    ros::spin();
}

// TODO add ability to cluster and generate vfs for objects here once board graph is ready

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_include");
    GamePlayer game_player;
    return 0;
}


// BoardConstructor::BoardConstructor ()
// {
//     // . Testing board utils
//     board_utils::loadCloudasMsg("/home/steven/game_theoretic_painting/src/models/clouds/revised/map.pcd", map_);
//     board_utils::loadCloudasMsg("/home/steven/game_theoretic_painting/src/models/clouds/revised/marked.pcd", marked_);

//     float map_discretization {3.0}; // 3 looks pretty good for the map we have
//     float repair_discretization {1.0};

//     OctreeData map_data;
//     map_data.name = "map";
//     board_utils::extractOctreeData(map_, map_discretization, map_data);

//     // OctreeData marked_data;
//     // marked_data.name = "marked";
//     // board_utils::extractOctreeData(marked_, repair_discretization, marked_data);

//     board_utils::saveOctreeDataClouds(map_data, "/home/steven/game_theoretic_painting/src/models/clouds/outputs/");
//     // board_utils::saveOctreeDataClouds(marked_data, "/home/steven/game_theoretic_painting/src/models/clouds/outputs/");

//     // . Graphs
//     SpatialGraph graph {board_utils::generateSpatialGraph(map_data)};
//     std::cout << "Finished constructing spatial graph!" << std::endl;

//     for (int i = 0; i < map_data.centroids.size(); i++) {
//         std::cout << "-----\n" << i << "\n-----" << std::endl;
//         std::cout << graph[i].x_pos << "\n" << graph[i].y_pos << "\n" << graph[i].z_pos << std::endl;
//         std::cout << graph[i].x_neg << "\n" << graph[i].y_neg << "\n" << graph[i].z_neg << std::endl;
//     }

//     // . Finally making board
//     Board board {board_utils::generateBoard(map_data, graph)};
//     std::cout << "Done constructing board!" << std::endl;

//     for (int i = 0; i < map_data.centroids.size(); i++) {
//         std::cout << "-----\n" << board[i].id << "\n-----" << std::endl;
//         std::cout << board[i].is_ground_level << std::endl;
//         std::cout << "gantry_edges: ";
//         for (int j = 0; j < board[i].gantry_edges.size(); j++) {
//             std::cout << board[i].gantry_edges[j] << ", ";
//         }
//         std::cout << std::endl;
//         std::cout << "quadruped_edges: ";
//         for (int j = 0; j < board[i].quadruped_edges.size(); j++) {
//             std::cout << board[i].quadruped_edges[j] << ", ";
//         }
//         std::cout << std::endl;
//         std::cout << "drone_edges: ";
//         for (int j = 0; j < board[i].drone_edges.size(); j++) {
//             std::cout << board[i].drone_edges[j] << ", ";
//         }
//         std::cout << std::endl;
//     }

//     // . Testing agents
//     // agents::Players players {agents::instantiatePlayers(1, 1, 1)};

//     // std::cout << players.drones[0].remaining_movement << std::endl;
//     // std::cout << players.drones[0].get_max_movement() << std::endl;
//     // players.drones[0].remaining_movement--;
//     // std::cout << players.drones[0].remaining_movement << std::endl;
//     // players.drones[0].reset_remaining_movement();
//     // std::cout << players.drones[0].remaining_movement << std::endl;

//     ros::spin();
// }
