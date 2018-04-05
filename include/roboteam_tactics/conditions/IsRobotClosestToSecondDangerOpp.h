#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class IsRobotClosestToSecondDangerOpp
 * \brief See YAML
 */
/*
 * Descr: Checks whether the ball can be intercepted during a duel. Returns Failure if opponent is in front of the ball, else it returns Success
 * Params: null
 */
    class IsRobotClosestToSecondDangerOpp : public Condition {
    public:
        IsRobotClosestToSecondDangerOpp(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

        Status Update() override;

        static VerificationMap required_params() {
            VerificationMap params;
            params["me"] = BBArgumentType::Int;
            return params;
        }

        std::string node_name() { return "IsRobotClosestToSecondDangerOpp"; }
    private:
        int robotID;
        int count = 0;
    } ;

}
