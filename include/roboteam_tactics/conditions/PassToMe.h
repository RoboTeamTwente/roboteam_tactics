#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class PassToMe
 * \brief See YAML
 */
/*
 * Descr: Checks whether a teammate chose to pass to me
 * Params:
 *   - resetParam:
 *       Type: Bool
 *       Descr: Whether to reset the passToRobot ros param to -1 after checking
 */
class PassToMe : public Condition {
public:
    PassToMe(std::string name, bt::Blackboard::Ptr blackboard);
    Status Update() override;
    
    std::string node_name() { return "PassToMe"; }
private:
};

}
