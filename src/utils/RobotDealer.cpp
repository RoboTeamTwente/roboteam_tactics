#include <vector>
#include <set>
#include <ros/ros.h>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"

namespace rtt {

std::set<int> RobotDealer::taken_robots;
std::set<int> RobotDealer::available_robots;
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

void RobotDealer::release_robot(int id) {
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

} // rtt
