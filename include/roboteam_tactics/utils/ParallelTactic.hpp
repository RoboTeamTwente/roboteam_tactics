#pragma once

#include <vector>

#include "roboteam_tactics/bt/Composite.hpp"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG ParallelTactic

namespace rtt {

class ParallelTactic : public bt::Composite {
public:
    ParallelTactic(bool successOnAll = true, bool failOnAll = true) : useSuccessFailPolicy(true), successOnAll(successOnAll), failOnAll(failOnAll) {}
    ParallelTactic(int minSuccess, int minFail) : minSuccess(minSuccess), minFail(minFail) {}

    std::vector<bt::Node::Status> lastStatus;

    void Initialize() override {
        RTT_DEBUGLN("Initializing Parallel Tactic");
        lastStatus.clear();
        for (auto child : children) {
            lastStatus.push_back(bt::Node::Status::Running);
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
            // Only run the child if it has not finished yet
            if (lastStatus.at(i) == bt::Node::Status::Running) {
                lastStatus.at(i) = child->Tick();
            } 

            if (lastStatus.at(i) == Status::Success) {
                totalSuccess++;
            }

            if (lastStatus.at(i) == Status::Failure) {
                totalFail++;
            }

            if (lastStatus.at(i) == Status::Invalid) {
                encounteredAnInvalid = true;
            }

            i++;
        }

        if (encounteredAnInvalid) {
            CallLeftoverTerminators();
            return Status::Failure;
        }

        if (totalSuccess >= minimumSuccess) {
            CallLeftoverTerminators();
            return Status::Success;
        }
        if (totalFail >= minimumFail) {
            CallLeftoverTerminators();
            return Status::Failure;
        }

        return Status::Running;
    }

    void CallLeftoverTerminators() {
        int i = 0;
        for (auto &child : children) {
            if (lastStatus.at(i) == Status::Running) {
                child->Terminate(Status::Running);
                child->Reset();
            }

            ++i;
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
