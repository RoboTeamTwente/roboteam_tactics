#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "unique_id/unique_id.h" 
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/StandByTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG StandByTactic

namespace rtt {

RTT_REGISTER_TACTIC(StandByTactic);

StandByTactic::StandByTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void StandByTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing StandByTactic \n");
    
    if (RobotDealer::get_available_robots().size() < 2) {
        RTT_DEBUG("Not enough robots, cannot initialize... \n");
        // TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();

    int firstRobotID = 0;
    int secondRobotID = 1;

    double firstRobotAngleGoal = 0.0;
    double secondRobotAngleGoal = 0.0;
    Vector2 firstRobotGoalPos(-4.4, -2.0);
    Vector2 secondRobotGoalPos(-4.4, -2.5);

    std::string ourSide;
    ros::param::get("our_side", ourSide);

    if (ourSide == "right") {
        firstRobotGoalPos.x = firstRobotGoalPos.x * -1;
        secondRobotGoalPos.x = secondRobotGoalPos.x * -1;
    }
    
    // delete_from_vector(robots, attackerID);
    claim_robots({firstRobotID, secondRobotID});

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Create the first robot role
    {
        roboteam_msgs::RoleDirective rd;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", firstRobotID);
        bb.SetInt("KEEPER_ID", 5);

        bb.SetDouble("GoToPos_A_xGoal", firstRobotGoalPos.x);
        bb.SetDouble("GoToPos_A_yGoal", firstRobotGoalPos.y);
        bb.SetDouble("GoToPos_A_angleGoal", firstRobotAngleGoal);
        bb.SetBool("GoToPos_A_avoidRobots", true);
        bb.SetBool("GoToPos_A_dribbler", false);

        // Create message
        rd.robot_id = firstRobotID;
        rd.tree = "qualification/StandByRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    // Create the first robot role
    {
        roboteam_msgs::RoleDirective rd;
        bt::Blackboard bb;

        // Set the robot ID
        bb.SetInt("ROBOT_ID", secondRobotID);
        bb.SetInt("KEEPER_ID", 5);

        bb.SetDouble("GoToPos_A_xGoal", secondRobotGoalPos.x);
        bb.SetDouble("GoToPos_A_yGoal", secondRobotGoalPos.y);
        bb.SetDouble("GoToPos_A_angleGoal", secondRobotAngleGoal);
        bb.SetBool("GoToPos_A_avoidRobots", true);
        bb.SetBool("GoToPos_A_dribbler", false);

        // Create message
        rd.robot_id = secondRobotID;
        rd.tree = "qualification/StandByRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }

    start = rtt::now();
}

bt::Node::Status StandByTactic::Update() {
    bool allSucceeded = true;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            allSucceeded &= status == bt::Node::Status::Success;
        } else {
            allSucceeded = false;
        }
    }

    if (allSucceeded) {
        RTT_DEBUGLN("Tactic success!");
        return Status::Success;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 25) {
        return Status::Failure;
    }

    return Status::Running;
}

} // rtt
