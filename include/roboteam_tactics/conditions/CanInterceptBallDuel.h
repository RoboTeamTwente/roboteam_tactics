#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class CanInterceptBallDuel
 * \brief See YAML
 */
/*
 * Descr: Checks whether the ball can be intercepted during a duel
 * Params: null
 */
class CanInterceptBallDuel : public Condition {
    public:
    CanInterceptBallDuel(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update() override;

    static VerificationMap required_params() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        return params;
    }

    std::string node_name() { return "CanInterceptBallDuel"; }
    private:
    int robotID;
    int count = 0;
} ;

}
