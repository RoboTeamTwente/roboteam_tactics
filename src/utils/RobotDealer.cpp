#include <vector>
#include <set>
#include <ros/ros.h>
#include <boost/optional.hpp>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG RobotDealer

namespace rtt {

std::set<int> RobotDealer::taken_robots;
std::set<int> RobotDealer::available_robots;
std::map<std::string, std::set<int>> RobotDealer::robot_owners;
int RobotDealer::keeper;
bool RobotDealer::keeper_available;

void RobotDealer::initialize_robots(int keeper, std::vector<int> ids) {
    taken_robots.clear();
    available_robots.clear();
    available_robots.insert(ids.begin(), ids.end());

    // Explicitly put namespace here s.t. shadowing does not mess up
    // the assignment
    RobotDealer::keeper = keeper;
    keeper_available = true;
}

std::vector<int> RobotDealer::get_available_robots() {
    return std::vector<int>(available_robots.begin(), available_robots.end());
}

int RobotDealer::get_keeper() {
    return keeper;
}

bool RobotDealer::get_keeper_available() {
    return keeper_available;
}

void RobotDealer::claim_robot(int id) {
    if (id == keeper) {
        if (!keeper_available) {
            ROS_ERROR("Keeper already taken! ID: %i", keeper);
        }

        keeper_available = false;
        return;
    } 

    if (taken_robots.find(id) != taken_robots.end()) {
        ROS_ERROR("Robot %d is already claimed!", id);
    }
    
    available_robots.erase(id);
    taken_robots.insert(id);
}

void RobotDealer::claim_robot_for_tactic(int id, std::string const & playName) {
    claim_robot(id);

    robot_owners[playName].insert(id);
}

void RobotDealer::claim_robot_for_tactic(std::vector<int> ids, std::string const & playName) {
    for (auto const id : ids) {
        claim_robot_for_tactic(id, playName);
    }
}

std::map<std::string, std::set<int>> const & RobotDealer::getRobotOwnerList() {
    return robot_owners;
}

void RobotDealer::printRobotDistribution() {
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

void RobotDealer::release_robot(int id) {
    RTT_DEBUGLN("Releasing robot %i", id);

    removeRobotFromOwnerList(id);

    if (id == keeper) {
        if (keeper_available) {
            ROS_ERROR("Goalkeeper was not claimed! ID: %i", keeper);
        }

        keeper_available = true;
        return;
    }

    if (available_robots.find(id) != available_robots.end()) {
        ROS_ERROR("Robot %d is already available!", id);
    }
    
    available_robots.insert(id);
    taken_robots.erase(id);

}

void RobotDealer::claim_robots(std::vector<int> ids) {
    for (int id : ids) {
        claim_robot(id);
    }
}

void RobotDealer::release_robots(std::vector<int> ids) {
    for (int id : ids) {
        release_robot(id);
    }
}

void RobotDealer::halt_override() {
    ROS_WARN("Overriding claims for all robots because of HALT");
    available_robots.insert(taken_robots.begin(), taken_robots.end());
    taken_robots.clear();
}

} // rtt
