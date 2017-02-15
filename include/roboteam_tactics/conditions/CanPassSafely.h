#pragma once

#include <boost/optional.hpp>

#include "roboteam_tactics/Parts.h"

namespace rtt {
    
class CanPassSafely : public Condition {

public:
    CanPassSafely(std::string name, bt::Blackboard::Ptr blackboard);  
    Status Update();
    
    std::string node_name() { return "CanPassSafely"; }
private:
};
    
} // rtt
