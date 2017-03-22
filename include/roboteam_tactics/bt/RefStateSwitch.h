#pragma once

#include <string>
#include <array>
#include <ros/ros.h>

#include "roboteam_tactics/bt.hpp"
#include "roboteam_utils/LastRef.h"
#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"

namespace rtt {

/**
 * \class RefStateSwitch
 * \brief Top-level node which selects the correct strategy tree to use based on the current ref state.
 */
class RefStateSwitch : public bt::Composite {    
    
public:
    /**
     * \brief The children which a RefStateSwitch should have. Mostly for error messages.
     */
    std::array<std::string, 19> const EXPECTED_CHILDREN = {{
        // TODO: When GCC gets its shit together, replace the doubly
        // braces with singly braces
        // (it's a bug: http://stackoverflow.com/questions/8192185/using-stdarray-with-initialization-lists)
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
    }};
    
    RefStateSwitch() : validated(false), last(-1) {}
    
    /**
     * \brief Checks whether this RefStateSwitch has all the children it should have.
     */
    bool isValid() const {
        return children.size() == EXPECTED_CHILDREN.size();
    }
    
    /**
     * \brief Asserts that this RefStateSwitch should be in a valid state, and prints an error message if it is not.
     */
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

        int cmd = (int) LastRef::get().command.command;

        auto child = children.at(cmd);

        if (last != cmd) {
            if (last != -1) {
                children.at(last)->Terminate(children.at(last)->getStatus());
            }

            child->Initialize();

            last = cmd;
        }

        return child->Update();
    }

    void Terminate(Status s) override {
        if (last != -1) {
            children.at(last)->Terminate(children.at(last)->getStatus());

            last = -1;
        }

        if (s == Status::Running) {
            setStatus(Status::Failure);
        }
    }
    
    std::string node_name() override { return "RefStateSwitch"; }
private:
    bool validated;
    int last;
};
    
}
