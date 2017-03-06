#include "roboteam_msgs/World.h"

#include "roboteam_tactics/conditions/BallOnTheirSide.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"

#include <string>

namespace rtt {

RTT_REGISTER_CONDITION(BallOnTheirSide);

BallOnTheirSide::BallOnTheirSide(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

bt::Node::Status BallOnTheirSide::Update() {
	roboteam_msgs::World world = LastWorld::get();
	roboteam_utils::Vector2 ballPos(world.ball.pos);
	
    std::string ourSide;
    ros::param::get("our_side", ourSide);
    if (ourSide == "left") {
        if (ballPos.x < 0) {
            // ROS_INFO("BallOnTheirSide false");
            return Status::Failure;
        } else {
            // ROS_INFO("BallOnTheirSide true");
            return Status::Success;
        }
    } else if (ourSide == "right") {
        if (ballPos.x > 0) {
            // ROS_INFO("BallOnTheirSide false");
            return Status::Failure;
        } else {
            // ROS_INFO("BallOnTheirSide true");
            return Status::Success;
        }
    }
    return Status::Invalid;
}

} // rtt
