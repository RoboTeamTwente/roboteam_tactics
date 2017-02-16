#pragma once

#include "Node.hpp"
#include <string>
#include <array>
#include "ros/ros.h"

#include "roboteam_utils/LastRef.h"
#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"

namespace rtt {

class RefStateSwitch : public bt::Composite {    
    
public:
    static constexpr std::array<const char*, 19> EXPECTED_CHILDREN = {
        "HALT=0",
        "STOP=1",
        "NORMAL_START=2",
        "FORCE_START=3",
        "PREPARE_KICKOFF_US=4",
        "PREPARE_KICKOFF_THEM=5",
        "PREPARE_PENALTY_US=6",
        "PREPARE_PENALTY_THEM=7",
        "DIRECT_FREE_US=8",
        "DIRECT_FREE_THEM=9",
        "INDIRECT_FREE_US=10",
        "INDIRECT_FREE_THEM=11",
        "TIMEOUT_US=12",
        "TIMEOUT_THEM=13",
        "GOAL_US=14",
        "GOAL_THEM=15",
        "BALL_PLACEMENT_US=16",
        "BALL_PLACEMENT_THEM=17",
        "NORMAL_PLAY=18"
    };
    
    RefStateSwitch() : validated(false) {}
    
    bool isValid() const {
        return children.size() == EXPECTED_CHILDREN.size();
    }
    
    void assertValid() const {
        if (!isValid()) {
            ROS_ERROR("Assertion Failed: RefStateSwitch expected %lu children, but got %lu. Here's a list of what it needs:",
                        EXPECTED_CHILDREN.size(), children.size());
            for (const std::string& s : EXPECTED_CHILDREN) {
                ROS_ERROR("%s", s.c_str());
            }
        }
    }
    
    void AddChild(Node::Ptr child) final override {
        if (!validated) {
            bt::Composite::AddChild(child);
        }
    }
    
    bt::Node::Status Update() override {
        if (!validated) {
            assertValid();
        }
        validated = true;
        return children.at((int) LastRef::get().command.command)->Update();
    }
    
    std::string node_name() override { return "RefStateSwitch"; }
private:
    bool validated;
};
    
}