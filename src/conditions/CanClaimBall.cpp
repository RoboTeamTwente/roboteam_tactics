#include "roboteam_tactics/conditions/CanClaimBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(CanClaimBall);

CanClaimBall::CanClaimBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status CanClaimBall::Update() {

    roboteam_msgs::World world = LastWorld::get();
    int robotID = GetInt("ROBOT_ID");
    bool canClaimBall = true;

    for (size_t i = 0; i < world.us.size(); i++) {
        int currentID = world.us.at(i).id;
        if (currentID != robotID) {
            std::string paramName = "robot" + std::to_string(currentID) + "/claimedBall";

            bool claimedBall;
            ros::param::getCached(paramName, claimedBall);
            if (claimedBall) {
                canClaimBall = false;
                break;
            }
        }
    } 

    if (canClaimBall) { return Status::Success; }

    return Status::Failure;
}
}