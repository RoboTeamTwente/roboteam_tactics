#include "roboteam_tactics/conditions/BallOnOurSide.h"

namespace rtt {

BallOnOurSide::BallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status BallOnOurSide::Update() {
    count++;
    std::cout << "Count: " << count << "\n";
    if (count == 3) {
        std::cout << "Ball on our side!\n";
        count = 0;
        return Status::Success;
    } else {
        std::cout << "Ball not on our side :(\n";
        return Status::Failure;
    }
}

} // rtt
