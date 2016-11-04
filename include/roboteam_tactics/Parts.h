#pragma once

#include <map>
#include <type_traits>
#include <vector>

#include "ros/ros.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/verifier.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_msgs/World.h"

namespace rtt {

class Skill : public Leaf {
public:

    Skill(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            , n{n}
            {}
    virtual ~Skill() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    ros::NodeHandle n;
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

class Tactic : public bt::Leaf {
    public:
    Tactic(bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            {}
    virtual ~Tactic() {}

    virtual void Initialize() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    virtual void Terminate(Status s) {
        RobotDealer::release_robots(get_claimed_robots());
        RobotDealer::release_role_nodes(get_claimed_role_nodes());
    }

    void claim_robot(int id) {
        if (claimed_robots.find(id) != claimed_robots.end()) {
            ROS_ERROR("Robot %d is already claimed by this tactic!\n", id);
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

    void claim_role_node() {
        claimed_role_nodes.insert(RobotDealer::claim_role_node()); 
    }

    void claim_role_nodes(size_t node_count) {
        for (size_t i = 0; i < node_count; i++) {
            claim_role_node();       
        }
    }

    std::vector<std::string> get_claimed_role_nodes() {
        return std::vector<std::string>(claimed_role_nodes.begin(), claimed_role_nodes.end());
    }

    private:
    std::set<int> claimed_robots;
    std::set<std::string> claimed_role_nodes;
} ;

}
