#include <vector>
#include <set>
#include <ros/ros.h>
#include <boost/optional.hpp>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"

#include "boost/interprocess/sync/scoped_lock.hpp"

#define RTT_CURRENT_DEBUG_TAG RobotDealer

namespace rtt {

std::set<int> RobotDealer::taken_robots;
// std::set<int> RobotDealer::available_robots;
std::map<std::string, std::set<int>> RobotDealer::robot_owners;
int RobotDealer::keeper;
bool RobotDealer::keeper_available = true;
boost::interprocess::interprocess_mutex RobotDealer::mutex;

#define LOCK_THIS_FUNCTION \
	boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>(RobotDealer::mutex);

// void RobotDealer::initialize_robots(int keeper, std::vector<int> ids) {
	// LOCK_THIS_FUNCTION
    // taken_robots.clear();
    // available_robots.clear();
    // available_robots.insert(ids.begin(), ids.end());

    // // Explicitly put namespace here s.t. shadowing does not mess up
    // // the assignment
    // RobotDealer::keeper = keeper;
    // keeper_available = true;

    // if (available_robots.find(keeper) != available_robots.end()) {
        // ROS_WARN("RobotDealer: Keeper was included in the available id vector!");
    // }
// }

std::vector<int> RobotDealer::getClaimedRobots() {
    return std::vector<int>(taken_robots.begin(), taken_robots.end());
}

void RobotDealer::setKeeper(int id) {
    keeper = id;
}

// std::vector<int> RobotDealer::get_available_robots() {
	// LOCK_THIS_FUNCTION
    // return std::vector<int>(available_robots.begin(), available_robots.end());
// }

int RobotDealer::get_keeper() {
    return keeper;
}

bool RobotDealer::get_keeper_available() {
    return keeper_available;
}

bool RobotDealer::claim_robot(int id) {
	LOCK_THIS_FUNCTION

    RTT_DEBUGLN("claim_robot with id: %i",id);
    if (id == keeper) {
        if (!keeper_available) {
            ROS_ERROR("Keeper already taken! ID: %i", keeper);
            return false;
        }

        keeper_available = false;
        return true;
    } 

    if (taken_robots.find(id) != taken_robots.end()) {
        ROS_ERROR("Robot %d is already claimed!", id);
        return false;
    }
    
    // if (available_robots.find(id) == available_robots.end()) {
        // ROS_ERROR("Tried to claim unavailable, unclaimed robot: %d", id);
        // return false;
    // }

    // available_robots.erase(id);
    taken_robots.insert(id);
    return true;
}

bool RobotDealer::release_robot(int id) {
	LOCK_THIS_FUNCTION
    RTT_DEBUGLN("Releasing robot %i", id);

    removeRobotFromOwnerList(id);

    if (id == keeper) {
        if (keeper_available) {
            ROS_ERROR("Goalkeeper was not claimed! ID: %i", keeper);
            return false;
        }

        keeper_available = true;
        return true;
    }

    // if (available_robots.find(id) != available_robots.end()) {
        // ROS_ERROR("Robot %d is already available!", id);
        // return false;
    // }

    if (taken_robots.find(id) == taken_robots.end()) {
    	ROS_ERROR("Tried to release an unclaimed robot: %d!", id);
    	return false;
    }

    // available_robots.insert(id);
    taken_robots.erase(id);
    return true;
}

bool RobotDealer::claim_robot_for_tactic(int id, std::string const & playName) {
    bool success = claim_robot(id);

    LOCK_THIS_FUNCTION
    if (success) {
    	robot_owners[playName].insert(id);
    }

    return success;
}

bool RobotDealer::claim_robot_for_tactic(std::vector<int> ids, std::string const & playName) {
    bool allClaimed = true;
	for (auto const id : ids) {
        allClaimed &= claim_robot_for_tactic(id, playName);
    }
	return allClaimed;
}

std::map<std::string, std::set<int>> const & RobotDealer::getRobotOwnerList() {
    return robot_owners;
}

void RobotDealer::printRobotDistribution() {
	LOCK_THIS_FUNCTION

    std::cout << "[RobotDistribution]\n";
    for (auto const & entry : robot_owners) {
        std::cout << entry.first << ":\n";
        for (auto const & id : entry.second) {
            std::cout << "\t- " << id << "\n";
        }
    }
}

/**
 * Remove robots from robot owner list.
 */
void RobotDealer::removeRobotFromOwnerList(int id) {
	LOCK_THIS_FUNCTION
    boost::optional<std::string> playToRemove;

    // For each robot set list...
    for (auto & entry : robot_owners) {
        // Get the set
        auto & robotSet = entry.second;
        // Check if the robot is in there
        auto robotIt = robotSet.find(id);
        if (robotIt != robotSet.end()) {
            // If so, erase it
            robotSet.erase(robotIt);

            // And if the set is then empty, mark it for removal from the map
            if (robotSet.size() == 0) {
                playToRemove = entry.first;
            }

            break;
        }
    }

    // If there was a set empty after removal, remove it from the map
    if (playToRemove) {
        robot_owners.erase(*playToRemove);
    }
}

bool RobotDealer::claim_robots(std::vector<int> ids) {
    bool allClaimed = true;
	for (int id : ids) {
        allClaimed &= claim_robot(id);
    }
	return allClaimed;
}

bool RobotDealer::release_robots(std::vector<int> ids) {
    bool allReleased = true;
	for (int id : ids) {
        allReleased &= release_robot(id);
    }
	return allReleased;
}

void RobotDealer::halt_override() {
	LOCK_THIS_FUNCTION
    ROS_WARN("Overriding claims for all robots because of HALT");
    // available_robots.insert(taken_robots.begin(), taken_robots.end());
    taken_robots.clear();
}

} // rtt
