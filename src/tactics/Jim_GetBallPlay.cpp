#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_GetBallPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "unique_id/unique_id.h" 

#define RTT_CURRENT_DEBUG_TAG Jim_GetBallPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_GetBallPlay);

Jim_GetBallPlay::Jim_GetBallPlay(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void Jim_GetBallPlay::Initialize() {
    tokens.clear();

    // RTT_DEBUG("Initializing Jim_GetBallPlay \n");

    roboteam_msgs::World world = LastWorld::get();

    if (RobotDealer::get_available_robots().size() < 1) {
        RTT_DEBUG("No robots available, cannot initialize... \n");
        //TODO: Want to pass failure here as well!
        return;
    }
    
    std::vector<int> robots = RobotDealer::get_available_robots();
    Vector2 ballPos = Vector2(world.ball.pos);
    int ballGetterID = get_robot_closest_to_point(robots, world, ballPos);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    RTT_DEBUGLN("GetBall robot: %i ", ballGetterID);


    // =============================
    // Initialize the Ball Getter
    // =============================
    {
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = ballGetterID;
        bt::Blackboard bb;
        claim_robot(ballGetterID);
        activeRobot = ballGetterID;

        bb.SetInt("ROBOT_ID", ballGetterID);
        bb.SetInt("KEEPER_ID", 5);

        bb.SetBool("GetBall_A_passToBestAttacker", true); 

        // Create message
        rd.tree = "rtt_jim/GetBallRole";
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

void Jim_GetBallPlay::ReleaseAllBots() {

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

    roboteam_msgs::RoleDirective rd;
    rd.robot_id = activeRobot;
    rd.tree = roboteam_msgs::RoleDirective::STOP_EXECUTING_TREE;
    pub.publish(rd);
    RobotDealer::release_robot(activeRobot);

    return;
}


bt::Node::Status Jim_GetBallPlay::Update() {

    // if (time_difference_milliseconds(start, now()).count() >= 1000) {
    //     Terminate(Status::Running);
    //     Initialize();
    // }
 
    for (auto token : tokens) {
        if (feedbacks.find(token) != feedbacks.end()) {
            Status status = feedbacks.at(token);
            if (status == Status::Success) {
                RTT_DEBUGLN("Jim_GetBallPlay Success!");
            }
            return status;
        }
    }

    return Status::Running;
}

} // rtt