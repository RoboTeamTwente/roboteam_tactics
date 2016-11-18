#include <vector>
#include <set>
#include <ros/ros.h>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"

namespace rtt {

std::set<int> RobotDealer::taken_robots;
std::set<int> RobotDealer::available_robots;

void RobotDealer::initialize_robots(std::vector<int> ids) {
    taken_robots.clear();
    available_robots.clear();
    available_robots.insert(ids.begin(), ids.end());
}

std::vector<int> RobotDealer::get_available_robots() {
    return std::vector<int>(available_robots.begin(), available_robots.end());
}

void RobotDealer::claim_robot(int id) {
    if (taken_robots.find(id) != taken_robots.end()) {
        ROS_ERROR("Robot %d is already claimed!", id);
    }
    
    available_robots.erase(id);
    taken_robots.insert(id);
}

void RobotDealer::release_robot(int id) {
    if (available_robots.find(id) != available_robots.end()) {
        ROS_ERROR("Robot %d is already available!", id);
    }
    
    taken_robots.erase(id);
    available_robots.insert(id);
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
