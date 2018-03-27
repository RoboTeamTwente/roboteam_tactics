#include "roboteam_tactics/conditions/PassToMe.h"

#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

RTT_REGISTER_CONDITION(PassToMe);

PassToMe::PassToMe(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status PassToMe::Update() {

    int robotID = GetInt("ROBOT_ID");
    int passToRobot;
    ros::param::getCached("passToRobot", passToRobot);
    if (passToRobot == robotID) {
        // ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", PassToMe detected");
        if (GetBool("resetParam")) {
            ros::param::set("passToRobot", -1);
            ROS_DEBUG_STREAM_NAMED("jelle_test1", "robot " << robotID << ", passToRobot param reset to -1 because resetParam is enabled in PassToMe condition");
        }
        return Status::Success;

    } else {
        return Status::Failure;
    }

}

} //rtt