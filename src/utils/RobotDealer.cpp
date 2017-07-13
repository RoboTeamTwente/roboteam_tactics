#include <vector>
#include <set>
#include <ros/ros.h>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/utils/debug_print.h"

#include "boost/interprocess/sync/scoped_lock.hpp"

#define RTT_CURRENT_DEBUG_TAG RobotDealer

namespace rtt {

std::set<int> RobotDealer::taken_robots;
std::set<int> RobotDealer::available_robots;
int RobotDealer::keeper;
bool RobotDealer::keeper_available;
boost::interprocess::interprocess_mutex RobotDealer::mutex;

#define LOCK_THIS_FUNCTION \
	boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>(RobotDealer::mutex);

void RobotDealer::initialize_robots(int keeper, std::vector<int> ids) {
	LOCK_THIS_FUNCTION
    taken_robots.clear();
    available_robots.clear();
    available_robots.insert(ids.begin(), ids.end());

    // Explicitly put namespace here s.t. shadowing does not mess up
    // the assignment
    RobotDealer::keeper = keeper;
    keeper_available = true;

    if (available_robots.find(keeper) != available_robots.end()) {
    	ROS_WARN("RobotDealer: Keeper was included in the available id vector!");
    }
}

std::vector<int> RobotDealer::get_available_robots() {
	LOCK_THIS_FUNCTION
    return std::vector<int>(available_robots.begin(), available_robots.end());
}

int RobotDealer::get_keeper() {
    return keeper;
}

bool RobotDealer::get_keeper_available() {
    return keeper_available;
}

bool RobotDealer::claim_robot(int id) {
	LOCK_THIS_FUNCTION
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
    
    if (available_robots.find(id) == available_robots.end()) {
    	ROS_ERROR("Tried to claim unavailable, unclaimed robot: %d", id);
    	return false;
    }

    available_robots.erase(id);
    taken_robots.insert(id);
    return true;
}

bool RobotDealer::release_robot(int id) {
	LOCK_THIS_FUNCTION
    RTT_DEBUGLN("Releasing robot %i", id);

    if (id == keeper) {
        if (keeper_available) {
            ROS_ERROR("Goalkeeper was not claimed! ID: %i", keeper);
            return false;
        }

        keeper_available = true;
        return true;
    }

    if (available_robots.find(id) != available_robots.end()) {
        ROS_ERROR("Robot %d is already available!", id);
        return false;
    }

    if (taken_robots.find(id) == taken_robots.end()) {
    	ROS_ERROR("Tried to release an unclaimed robot: %d!", id);
    	return false;
    }
    
    available_robots.insert(id);
    taken_robots.erase(id);
    return true;
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
    available_robots.insert(taken_robots.begin(), taken_robots.end());
    taken_robots.clear();
}

} // rtt
