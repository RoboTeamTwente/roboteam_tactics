#include "roboteam_tactics/conditions/BallOnOurSide.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(BallOnOurSide);

BallOnOurSide::BallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status BallOnOurSide::Update() {
	roboteam_msgs::World world = LastWorld::get();
	roboteam_msgs::Vector2f ballPos = world.ball.pos;
	
	if (ballPos.x > 0) {
        std::cout << "Ball on our side!\n";
        count = 0;
        return Status::Success;
    } else {
        std::cout << "Ball not on our side :(\n";
        return Status::Failure;
    }
}

} // rtt
