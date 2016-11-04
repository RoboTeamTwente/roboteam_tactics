#include <vector>
#include <set>
#include <ros/ros.h>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"

namespace rtt {

std::set<int> RobotDealer::taken_robots;
std::set<int> RobotDealer::available_robots;

void initialize_robots(std::vector<int> ids) {

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

std::set<std::string> RobotDealer::taken_role_nodes;
std::set<std::string> RobotDealer::available_role_nodes;

void RobotDealer::initialize_role_nodes() {
    available_role_nodes.clear();
    auto workers = getNodesSubscribedTo(getMyNamespace() + "role_directive");

    available_role_nodes.insert(workers.begin(), workers.end());
}

std::string RobotDealer::claim_role_node() {
    std::string node = *available_role_nodes.begin();

    if (taken_role_nodes.find(node) != taken_role_nodes.end()) {
        ROS_ERROR("Role node %s is already taken!\n", node.c_str());
    }

    available_role_nodes.erase(node);
    taken_role_nodes.insert(node);
    return node;
}

std::vector<std::string> RobotDealer::claim_role_nodes(size_t n) {
    std::vector<std::string> nodes;
    for (size_t i = 0; i < n; i++) {
        nodes.push_back(claim_role_node());
    }
    return nodes;
}

void RobotDealer::release_role_node(std::string id) {
    if (taken_role_nodes.find(id) != taken_role_nodes.end()) {
        ROS_ERROR("Role node %s was not taken!\n", id.c_str());
    }

    taken_role_nodes.erase(id);
    available_role_nodes.insert(id);
}

void RobotDealer::release_role_nodes(std::vector<std::string> ids) {
    for (auto id : ids) {
        release_role_node(id);
    }
}

} // rtt
