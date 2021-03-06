#include <string>

#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"

#include "roboteam_tactics/Parts.h"

namespace rtt {

///////////
// Skill //
///////////

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        : Leaf(name, blackboard)
        {}
Skill::~Skill() {}

bt::Node::Status Skill::Update() {
    return Status::Invalid;
}

///////////////
// Condition //
///////////////

Condition::Condition(std::string name, bt::Blackboard::Ptr blackboard)
        : Leaf(name, blackboard)
        {}

Condition::~Condition() {
    // Nothing to di here
}

bt::Node::Status Condition::Update() {
    return Status::Invalid;
}

/**
 * @brief Returns a set of world states in which this Condition should succeed.
 * For use in testing. See test/ConditionsTest.cpp
 */
std::vector<roboteam_msgs::World> Condition::success_states() const {
    return std::vector<roboteam_msgs::World>();
}

/**
 * @brief Returns a set of world states in which this Condition should fail.
 * For use in testing. See test/ConditionsTest.cpp
 */
std::vector<roboteam_msgs::World> Condition::fail_states() const {
    return std::vector<roboteam_msgs::World>();
}

////////////
// Tactic //
////////////

// Enable RTT_DEBUG for Tactic
#define RTT_CURRENT_DEBUG_TAG Tactic

Tactic::Tactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Leaf(name, blackboard)
        {}

Tactic::~Tactic() {}

void Tactic::Initialize() {}

bt::Node::Status Tactic::Update() {
    return Status::Invalid;
}

void Tactic::Terminate(Status) {
    auto robots = get_claimed_robots();

    auto& pub = GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    roboteam_msgs::RoleDirective directive;
    directive.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;

    // Stop every robot
    for (auto robotID : robots) {
        directive.robot_id = robotID;
        pub.publish(directive);
    }

    RTT_DEBUGLN("Releasing tactic robots!\n");

    RobotDealer::release_robots(robots);
    claimed_robots.clear();

    if (getStatus() == bt::Node::Status::Running) {
        setStatus(bt::Node::Status::Failure);
    }
}

bool Tactic::claim_robot(int id) {
    if (claimed_robots.find(id) != claimed_robots.end()) {
        ROS_ERROR("Robot %d is already claimed by tactic %s!\n", id, name.c_str());
        for (const int ROBOT_ID : claimed_robots) {
            ROS_ERROR("Claimed robot: %d", ROBOT_ID);
        }
    }

    claimed_robots.insert(id);

    return RobotDealer::claim_robot_for_tactic(id, name);
}

bool Tactic::claim_robots(std::vector<int> ids) {
	bool allClaimed = true;
    for (int id : ids) {
        allClaimed &= claim_robot(id);
    }
    return allClaimed;
}

std::vector<int> Tactic::get_claimed_robots() {
    return std::vector<int>(claimed_robots.begin(), claimed_robots.end());
}

bool Tactic::is_claimed(int id) {
    return claimed_robots.find(id) != claimed_robots.end();
}

bool Tactic::release_robot(int id) {
	if (RobotDealer::release_robot(id)) {
		claimed_robots.erase(id);
		return true;
	}
	return false;
}

bool Tactic::release_robots(std::vector<int> ids) {
	bool allReleased = true;
	for (int id : ids) {
		allReleased &= release_robot(id);
	}
	return allReleased;
}

} // rtt
