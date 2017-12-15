#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Geneva.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG Geneva

namespace rtt {

RTT_REGISTER_SKILL(Geneva);

Geneva::Geneva(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard){
        cycleCounter = 0;
        }


void Geneva::Initialize() {
    cycleCounter = 0;
}


bt::Node::Status Geneva::Update() {

    cycleCounter++;
    if(cycleCounter > 20) {
        return bt::Node::Status::Success;
    }

        roboteam_msgs::World world = LastWorld::get();
        int robotID = blackboard->GetInt("ROBOT_ID");

        boost::optional <roboteam_msgs::WorldRobot> robotPointer = getWorldBot(robotID);
        roboteam_msgs::WorldRobot robot;
    //    if (robotPointer) {
//        robot = *robotPointer;
//    } else {
//        ROS_WARN("Geneva: Robot not found");
//        return Status::Failure;
//    }

    double genevaState;
    if (HasInt("genevaState")) {
      genevaState = GetInt("genevaState");
        std::cout << "HasInt succeeded" << std::endl;
    } else {
      genevaState = 0;
        std::cout << "HasInt failed" << std::endl;
    }

    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.geneva_state = genevaState;

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);


    return Status::Running;

}

} // rtt
