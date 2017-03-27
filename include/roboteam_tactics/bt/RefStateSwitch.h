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
    RefStateSwitch() : validated(false), last(-1) {
        std::cout << "Initializing last!\n";
    }
    
    /**
     * \brief Checks whether this RefStateSwitch has all the children it should have.
     */
    bool isValid() const {
        return children.size() == REF_STATE_COUNT;
    }
    
    /**
     * \brief Asserts that this RefStateSwitch should be in a valid state, and prints an error message if it is not.
     */
    void assertValid() const {
        if (!isValid()) {
            ROS_ERROR("Assertion Failed: RefStateSwitch expected %lu children, but got %lu.",
                        REF_STATE_COUNT, children.size());
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
            ROS_INFO("Received referee command %d", cmd);
            std::cout << "Last: " << last << "\n";
            last = cmd;
        }

        return child->Update();
    }

    void Terminate(Status s) override {
        std::cout << "Terminating RSS!\n";

        if (last != -1) {
            auto& child = children.at(last);

            // Only terminate if the child did not do so properly itself
            if (child->getStatus() == bt::Node::Status::Running) {
                child->Terminate(bt::Node::Status::Running);
            }

            last = -1;
        }

        // Consider the node failed if it did not properly finish
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
