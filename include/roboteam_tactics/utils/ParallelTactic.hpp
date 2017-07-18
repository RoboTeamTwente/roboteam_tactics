#pragma once

#include <vector>

#include "roboteam_tactics/bt/Composite.hpp"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "ros/ros.h"

#define RTT_CURRENT_DEBUG_TAG ParallelTactic

namespace rtt {

class ParallelTactic : public bt::Composite {
public:
    ParallelTactic(bool successOnAll = true, bool failOnAll = true) : useSuccessFailPolicy(true), successOnAll(successOnAll), failOnAll(failOnAll) {}
    ParallelTactic(int minSuccess, int minFail) : minSuccess(minSuccess), minFail(minFail) {}

    std::vector<bool> failedOrSucceeded;

    void Initialize() override {
        RTT_DEBUGLN("Initializing Parallel Tactic");
        failedOrSucceeded.clear();
        for (auto child : children) {
            failedOrSucceeded.push_back(false);
        }
    }

    Status Update() override {
        int minimumSuccess = minSuccess;
        int minimumFail = minFail;

        if (useSuccessFailPolicy) {
            if (successOnAll) {
                minimumSuccess = children.size();
            }
            else {
                minimumSuccess = 1;
            }

            if (failOnAll) {
                minimumFail = children.size();
            }
            else {
                minimumFail = 1;
            }
        }

        int totalSuccess = 0;
        int totalFail = 0;
        bool encounteredAnInvalid = false;

        int i = 0;
        for (auto &child : children) {
            if (!failedOrSucceeded.at(i)) {
                child->Tick();
            }

            Node::Status s = child->getStatus();

            if (s != Status::Running) {
                failedOrSucceeded.at(i) = true;
            }

            if (s == Status::Success) {
                totalSuccess++;
            }

            if (s == Status::Failure) {
                totalFail++;
            }

            if (s == Status::Invalid) {
                encounteredAnInvalid = true;
            }

            i++;
        }

        auto leftovers = RobotDealer::get_available_robots();
        if (leftovers.size() > 0) {
        	ROS_WARN("ParellelTactic: %lu robots remain unclaimed", leftovers.size());
        }

        if (encounteredAnInvalid) {
            return Status::Failure;
        }

        if (totalSuccess >= minimumSuccess) {
            return Status::Success;
        }
        if (totalFail >= minimumFail) {
            return Status::Failure;
        }

        return Status::Running;
    }

    void Terminate(Status s) override {
        for (auto &child : children) {
            if (child->getStatus() == Status::Running) {
                child->Terminate(Status::Running);
            }
        }

        if (s == Status::Running) {
            setStatus(Status::Failure);
        }

        return;
    }
    
    using Ptr = std::shared_ptr<ParallelTactic>;

private:
    bool useSuccessFailPolicy = false;
    bool successOnAll = true;
    bool failOnAll = true;
    int minSuccess = 0;
    int minFail = 0;
};

static ParallelTactic::Ptr MakeParallelTactic()
{
    return std::make_shared<ParallelTactic>();
}

}

// Disable debug tag to prevent spilling in cpp files
#undef RTT_CURRENT_DEBUG_TAG
