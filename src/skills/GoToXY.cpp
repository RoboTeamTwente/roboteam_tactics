#include "roboteam_tactics/skills/GoToXY.h"

namespace rtt {

GoToXY::GoToXY(Aggregator& aggregator, bt::Blackboard::Ptr blackboard)
        : Skill(aggregator, blackboard) {}

bt::Node::Status GoToXY::Update() {
    std::cout << "Going to XY...\n";
    return Status::Running;
}

} // rtt
