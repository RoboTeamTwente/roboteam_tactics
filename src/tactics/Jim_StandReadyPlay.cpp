#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_StandReadyPlay.h"
#include "roboteam_tactics/tactics/Jim_MultipleDefendersPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "unique_id/unique_id.h" 

#define RTT_CURRENT_DEBUG_TAG Jim_StandReadyPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_StandReadyPlay);

Jim_StandReadyPlay::Jim_StandReadyPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void Jim_StandReadyPlay::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing Jim_StandReadyPlay \n");

    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("No robots available, cannot initialize... \n");
        //TODO: Want to pass failure here as well!
        return;
    }
    
    roboteam_msgs::World world = LastWorld::get();
    std::vector<int> robots = RobotDealer::get_available_robots();
    int keeper = RobotDealer::get_keeper();
    // Vector2 keeperPos = Vector2(-4.0, -0.0);
    Vector2 centerPos = Vector2(-2.0, 0.0);
    Vector2 posOnCircle = Vector2(-0.6, 0.0);
    Vector2 keeperPos = centerPos + posOnCircle;

    ROS_INFO_STREAM("n_robots: " << robots.size());

    std::vector<Vector2> posList;

    for (size_t i = 0; i < robots.size(); i++) {
        posList.push_back(centerPos + posOnCircle.rotate( (i + 1) *  2*M_PI / (robots.size() + 1) ));
    }

    std::vector<int> closestRobots = Jim_MultipleDefendersPlay::getClosestRobots(robots, posList, world);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();


    // Initialize Keeper
    {
        claim_robot(keeper);

        roboteam_msgs::RoleDirective rd;
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", keeper);
        bb.SetInt("KEEPER_ID", keeper);

        bb.SetDouble("GoToPos_A_xGoal", keeperPos.x);
        bb.SetDouble("GoToPos_A_yGoal", keeperPos.y);
        bb.SetDouble("GoToPos_A_angleGoal", 0.0);

        // Create message
        rd.robot_id = keeper;
        rd.tree = "rtt_jim/StandReadyRole";
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        // Send to rolenode
        pub.publish(rd);
    }


    // Initialize the other players
    for (size_t i = 0; i < closestRobots.size(); i++) {
        ROS_INFO_STREAM("i: " << i << " robotID: " << closestRobots.at(i));

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

        // Create message
        rd.robot_id = closestRobots.at(i);
        rd.tree = "rtt_jim/StandReadyRole";
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


bt::Node::Status Jim_StandReadyPlay::Update() {
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
        RTT_DEBUGLN("Jim_StandReadyPlay succeeded!");
        return Status::Success;
    }

    auto duration = time_difference_seconds(start, now());
    if (duration.count() >= 25) {
        return Status::Failure;
    }

    return Status::Running;
}

} // rtt