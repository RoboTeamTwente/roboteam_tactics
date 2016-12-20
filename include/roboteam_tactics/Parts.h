#pragma once

#include <map>
#include <type_traits>
#include <vector>
#include <ros/ros.h>

#include "roboteam_utils/constants.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/verifier.h"
// #include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_tactics/utils/debug_print.h"

namespace rtt {

class Skill : public Leaf {
public:

    Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            {}
    virtual ~Skill() {}

    virtual Status Update() {
        return Status::Invalid;
    }
} ;

class Condition : public Leaf {
	public:
	Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            {}
    virtual ~Condition() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    /**
     * @brief Returns a set of world states in which this Condition should succeed.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> success_states() const {
        return std::vector<roboteam_msgs::World>();
    }

    /**
     * @brief Returns a set of world states in which this Condition should fail.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> fail_states() const {
        return std::vector<roboteam_msgs::World>();
    }
} ;

// Enable RTT_DEBUG for Tactic
#define RTT_CURRENT_DEBUG_TAG Tactic

class Tactic : public Leaf {
    public:
    Tactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            {}
    virtual ~Tactic() {}

    virtual void Initialize() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    virtual void Terminate(Status s) {
        auto robots = get_claimed_robots();

        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

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
    }

    void claim_robot(int id) {
        if (claimed_robots.find(id) != claimed_robots.end()) {
            ROS_ERROR("Robot %d is already claimed by this tactic!\n", id);
            for (const int ROBOT_ID : claimed_robots) {
                ROS_ERROR("Claimed robot: %d", ROBOT_ID);
            }
        }

        claimed_robots.insert(id);

        RobotDealer::claim_robot(id);
    }

    void claim_robots(std::vector<int> ids) {
        for (int id : ids) {
            claim_robot(id);
        }
    }

    std::vector<int> get_claimed_robots() {
        return std::vector<int>(claimed_robots.begin(), claimed_robots.end());
    }

    bool is_claimed(int id) {
        return claimed_robots.find(id) != claimed_robots.end();
    }

    private:
    std::set<int> claimed_robots;
} ;

// Make sure sources that include this are not troubled by the def
#undef RTT_CURRENT_DEBUG_TAG

}
