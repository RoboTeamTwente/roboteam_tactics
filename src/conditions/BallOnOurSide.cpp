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
	roboteam_utils::Vector2 ballPos(world.ball.pos);
	
    std::string ourSide;
    ros::param::get("our_side", ourSide);
    if (ourSide == "left") {
        if (ballPos.x < 0) {
            // ROS_INFO("BallOnOurSide true");
            return Status::Success;
        } else {
            // ROS_INFO("BallOnOurSide false");
            return Status::Failure;
        }
    } else if (ourSide == "right") {
        if (ballPos.x > 0) {
            // ROS_INFO("BallOnOurSide true");
            return Status::Success;
        } else {
            // ROS_INFO("BallOnOurSide false");
            return Status::Failure;
        }
    }
    return Status::Invalid;
}

} // rtt
