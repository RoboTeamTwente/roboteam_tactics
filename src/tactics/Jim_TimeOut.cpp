#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_TimeOut.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "unique_id/unique_id.h" 

#define RTT_CURRENT_DEBUG_TAG Jim_TimeOut

namespace rtt {

RTT_REGISTER_TACTIC(Jim_TimeOut);

Jim_TimeOut::Jim_TimeOut(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}


// void Jim_TimeOut::Initialize() {
void Jim_TimeOut::Initialize() {   
    tokens.clear();

    RTT_DEBUG("Initializing Jim_TimeOut \n");

    if (getAvailableRobots().size() < 1) {
        RTT_DEBUG("No robots available, cannot initialize... \n");
        //TODO: Want to pass failure here as well!
        return;
    }
    
    roboteam_msgs::World world = LastWorld::get();
    std::vector<int> robots = getAvailableRobots();
    int keeper = RobotDealer::get_keeper();
    // robots.push_back(keeper);
    Vector2 keeperPos(LastWorld::get_our_goal_center().x + 0.5, 0.0);

    double distanceBetweenRobots = 0.3;

    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    std::vector<Vector2> posList;

    for (size_t i = 0; i < robots.size(); i++) {
        Vector2 targetPos(-(field.field_length / 2) + 0.3, ((double) i + 0.5 - (double) robots.size()/2) * distanceBetweenRobots);
        ROS_INFO_STREAM("position: " << targetPos);
        posList.push_back(targetPos);
    }

    std::vector<int> closestRobots = Jim_MultipleDefendersPlay::assignRobotsToPositions(robots, posList, world);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    // Initialize the keeper
    {
        claim_robot(keeper);

        roboteam_msgs::RoleDirective rd;
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", keeper);
        bb.SetInt("KEEPER_ID", keeper);

        bb.SetDouble("GoToPos_A_xGoal", keeperPos.x);
        bb.SetDouble("GoToPos_A_yGoal", keeperPos.y);
        bb.SetDouble("GoToPos_A_angleGoal", 0.0);
        bb.SetDouble("GoToPos_A_maxVelocity",STOP_STATE_MAX_VELOCITY);
        bb.SetBool("GoToPos_A_avoidRobots", true);
        // bb.SetBool("GoToPos_A_enterDefenseAreas", true);

        // Create message
        rd.robot_id = keeper;
        rd.tree = "rtt_jim/GoToPosRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }



    // Initialize the roles
    for (size_t i = 0; i < closestRobots.size(); i++) {

        if (i >= posList.size()) {
            break;
        }

        claim_robot(closestRobots.at(i));

        roboteam_msgs::RoleDirective rd;
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", closestRobots.at(i));
        bb.SetInt("KEEPER_ID", keeper);

        bb.SetDouble("GoToPos_A_xGoal", posList.at(i).x);
        bb.SetDouble("GoToPos_A_yGoal", posList.at(i).y);
        bb.SetDouble("GoToPos_A_angleGoal", 0.0);
        bb.SetDouble("GoToPos_A_maxVelocity",STOP_STATE_MAX_VELOCITY);
        bb.SetBool("GoToPos_A_avoidRobots", true);
        // bb.SetBool("GoToPos_A_enterDefenseAreas", true);

        // Create message
        rd.robot_id = closestRobots.at(i);
        rd.tree = "rtt_jim/GoToPosRole";
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


bt::Node::Status Jim_TimeOut::Update() {
    bool allSucceeded = true;

    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (status == bt::Node::Status::Running) {
                ROS_INFO("running!");
            }
            allSucceeded &= status == bt::Node::Status::Success;
        } else {
            allSucceeded = false;
        }
    }

    if (allSucceeded) {
        RTT_DEBUGLN("Jim_TimeOut succeeded!");
        return Status::Success;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 25) {
        return Status::Failure;
    }

    return Status::Running;
}

} // rtt
