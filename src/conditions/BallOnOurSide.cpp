#include "roboteam_msgs/World.h"

#include "roboteam_tactics/conditions/BallOnOurSide.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"

#include <string>

namespace rtt {

RTT_REGISTER_CONDITION(BallOnOurSide);

BallOnOurSide::BallOnOurSide(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status BallOnOurSide::Update() {
	roboteam_msgs::World world = LastWorld::get();
	Vector2 ballPos(world.ball.pos);
	
    if (ballPos.x < 0) {
        if (HasBool("ourSide") && !GetBool("ourSide")) {
            return Status::Failure;
        } else {
            return Status::Success;
        }
    } else {
        if (HasBool("ourSide") && !GetBool("ourSide")) {
            return Status::Success;
        } else {
            return Status::Failure;
        }
    } 
    
    return Status::Invalid;
}

} // rtt
