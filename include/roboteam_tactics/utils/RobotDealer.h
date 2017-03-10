#pragma once

#include <set>
#include <string>
#include <vector>

namespace rtt {

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
    static std::set<int> taken_robots;
    static std::set<int> available_robots;
    static int keeper;
    static bool keeper_available;
} ;

} // rtt
