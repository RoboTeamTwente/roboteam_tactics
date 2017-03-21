#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class BallOnOurSide
 * \brief See YAML
 */
/*
 * Descr: Checks whether the ball is on our side of the field.
 * Params: null
 */
class BallOnOurSide : public Condition {
    public:
    BallOnOurSide(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update() override;

    std::string node_name() { return "BallOnOurSide"; }
    private:
    int count = 0;
} ;

}
