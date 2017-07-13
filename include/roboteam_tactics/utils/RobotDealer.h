#pragma once

#include <set>
#include <string>
#include <vector>
#include <map>

namespace rtt {

// For friend declaration
class HaltTactic;
    
/**
 * \class RobotDealer
 * \brief Keeps track of which robots have been 'claimed' by tactics
 */    
class RobotDealer {
public:
    /**
     * \brief Initialize the RobotDealer with a set of ids, and a keeper
     */
    static void initialize_robots(int keeper, std::vector<int> ids);
    
    /**
     * \brief Gets all unclaimed robot ids
     */
    static std::vector<int> get_available_robots();
    
    /**
     * \brief Gets the id of the designated keeper
     */
    static int get_keeper();
    
    /**
     * \brief Checks whether or not the keeper has been claimed by a tactic
     */
    static bool get_keeper_available();
    
    /**
     * \brief Marks the given id as claimed
     */
    static void claim_robot(int id);

    /**
     * Claims a robot for a tactic while simultanously also adding it to a list
     * of owners.
     */
    static void claim_robot_for_tactic(int id, std::string const & playName);

    /**
     * Claims a bunch of robots under the name of the given tactic.
     */
    static void claim_robot_for_tactic(std::vector<int> ids, std::string const & playName);

    /**
     * Returns a list of mappings from a tactics name to the set of robots that it owns
     * at the moment.
     */
    static std::map<std::string, std::set<int>> const & getRobotOwnerList();
    
    /**
     * \brief Marks the given id as unclaimed
     */
    static void release_robot(int id);
    
    /**
     * \brief Marks the given ids as claimed
     */
    static void claim_robots(std::vector<int> ids);
    
    /**
     * \brief Marks the given ids as unclaimed
     */
    static void release_robots(std::vector<int> ids);

private:

    /**
     * Only removes the robot from the owner list, but not from the actual
     * taken/available lists. Used internally.
     */
    static void removeRobotFromOwnerList(int id);

    static std::set<int> taken_robots;
    static std::set<int> available_robots;
    static std::map<std::string, std::set<int>> robot_owners;
    static int keeper;
    static bool keeper_available;
    static void printRobotDistribution();
    
    friend class HaltTactic;
    static void halt_override();
} ;

} // rtt
