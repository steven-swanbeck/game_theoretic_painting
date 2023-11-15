#ifndef GAME_MANAGER_H
#define GAME_MANAGER_H

#include "coverage_contest/common.h"
#include "coverage_contest/point.h"
#include "board.hpp"
#include "agents.hpp"

/** @class GameManager
 * @brief coordinates player turns and tracks game state, contains relevant game dynamics
 * @var GameManager::board_
 * member 'board_' contains the game state that is updated as the game progresses
 * @var GameManager::party_
 * member 'party_' contains the player info and attributes that are updated as the gram progresses
 * @var GameManager::player_turn_
 * member 'player_turn_' identifies the currently playing player
 * @var GameManager::total_turns_
 * member 'total_turns_' tracks the total number of full cycles of all players playing that have passed during the game episode
 */
class GameManager
{
public:
    GameManager (bool log_level=true);
    GameManager (Board board, agents::Party party, int player_turn, bool log_level=false);

    /** Plays random valid moves for each player until a terminal state is reached
     * @brief plays a game randomly to completion
     */
    void playRandomGame ();

    /** Randomly generates and executes a turn for the current player and updates game state accordingly 
     * @brief executes a random turn for the current player
     */
    void takeRandomTurn ();

    /** Randomly generates and executes a set number of individual turns into the future
     * @brief executes a random number of turns
     */
    void playToDepth (const int &depth);

    /** Given a set of candiate turns, randomly selects one and plays it
     * @brief executes a turn randomly chosen from a set of options
     * @param candidates TurnOptions type, set of possible valid turns
     */
    void playRandomMove (TurnOptions candidates);

    /** Plays a given turn, adjusting robot params and values accordingly  
     * @brief executes a turn
     * @param sequence TurnSequence type, queue of actions to execute
     */
    void playSequence (TurnSequence &sequence);

    /** Monitors for terminal state after each turn
     * @brief looks through RepairBoard and checks if any spots have yet to be covered
     * @return bool if game has reached terminal state
     */
    bool isOver ();

    /** Returns ids of winner(s) after a terminal board state has been reached
     * @brief finds players with highest scores in party
     * @return vector of id's of player(s) with highest score
     */
    std::vector<std::string> determineWinners ();

    /** Instantiates the party for the current game
     * @brief uses board_utils::generateBoard to generate a board from a given set of point clouds
     * @param move_dir location on disk from which to load cloud to generate movement graph
     * @param move_discretization voxel size for movement graph
     * @param repair_dir location on disk from which to load cloud to generate repair graph
     * @param repair_discretization voxel size for repair graph
     */
    void instantiateBoard (const std::string &move_dir, const float &move_discretization, const std::string &repair_dir, const float &repair_discretization);
    
    /** Instantiates the party for the current game
     * @brief uses agents::instantiatePlayers to generate a party of players
     * @param num_drones number of drone players
     * @param num_quadrupeds number of quadruped players
     * @param num_gantries number of gantry players
     * @param starting_position node id in spatial graph from which play begins for all players
     */
    void instantiatePlayers (const int &num_drones, const int &num_quadrupeds, const int &num_gantries, int starting_position=-1);
    
    /** Generates random playing turn order for the party
     * @brief uses agents::randomShufflePlayingOrder on instantiated party to generate a random turn order
     */
    void generateTurnOrder ();
    
    /** Cycles playing order
     * @brief manages playing order of current game
     * @return std::string type, id of next playing player
     */
    std::string startNext ();
    
    /** Returns unique id of currently playing robot
     * @brief returns id of playing robot
     * @return std::string type, id of current player
     */
    std::string playingNow ();
    
    /** Manages battery attribute of the currently playing robot
     * @brief checks battery attribute, decays it, and forces turn-based resting for the playing robot
     * @return bool type, whether robot has sufficient battery to play this turn
     */
    bool hasSufficientBattery ();
    
    /** Lists all possible actions that can be taken at a particular graph node for currently playing robot at its current location
     * @brief list moves that can be taken at a particular location given robot-specific movement constraints
     * @return MoveOptions type, as vector of Action types
     */
    MoveOptions listMovesfromNode ();
    
    /** Lists all possible actions that can be taken at a particular graph node for currently playing robot
     * @brief list moves that can be taken at a particular location given robot-specific movement constraints
     * @param index int type, denotes node id for which moves should be generated
     * @return MoveOptions type, as vector of Action types
     */
    MoveOptions listMovesfromNode (const int &index);
    
    /** Lists all possible actions that can be taken at the current location of a robot accounting for that robot's attributes
     * @brief list moves that can be taken at a particular location given robot-specific movement constraints and decaying attributes
     * @param player agents::Robot type with assigned relevant attributes
     * @param board RepairBoard type tracking areas that still need to be repaired
     * @return MoveOptions type, as vector of Action types
     */
    MoveOptions listMovesfromNodeConstrained (agents::Robot &player, RepairBoard &board);
    
    /** Generates a random possible turn for the current player accounting for the current game state
     * @brief randomly generate a move for the current player given the current game state
     * @return TurnSequence type, with a queue of Action types
     */
    TurnSequence generateRandomTurn ();
    
    /** Generates a number of random possible turns for the current player accounting for the current game state
     * @brief randomly generate moves for the current player given the current game state
     * @param num_moves the number of random candidate moves that should be generated
     * @return TurnOptions, vector of TurnSequence types, each with a queue of Action types
     */
    TurnOptions generateRandomTurns (const int num_moves);

    // Testing functions
    void printMovesfromState (int state);
    void testRandomTurns (int num);

    Board board_;
    agents::Party party_;
    int player_turn_;

private:
    int total_turns_;
    bool log_level_;
};

#endif // GAME_MANAGER_H
