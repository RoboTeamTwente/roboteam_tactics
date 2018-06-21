#include <algorithm>
#include <memory>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_tactics/tactics/Jim_PenaltyPlay.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "unique_id/unique_id.h" 

#define ROS_LOG_NAME "plays.Jim_PenaltyPlay"
#define RTT_CURRENT_DEBUG_TAG Jim_PenaltyPlay

namespace rtt {

RTT_REGISTER_TACTIC(Jim_PenaltyPlay);

Jim_PenaltyPlay::Jim_PenaltyPlay(std::string name, bt::Blackboard::Ptr blackboard) : Tactic(name, blackboard){}

void Jim_PenaltyPlay::Initialize() {
    tokens.clear();
    success = true;
    inShootout = false;

    const roboteam_msgs::World& world = LastWorld::get();

    // Check if ball is on our side, which would mean shootout
    if(world.ball.pos.x < 0){
        ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "PENALTY SHOOTOUT DETECTED! Initializing..");
        inShootout = true;
        Jim_PenaltyPlay::initShootout();
        return;
    }


    if (getAvailableRobots().size() < 1) {
        ROS_WARN_NAMED(ROS_LOG_NAME, "No robots available, cannot initialize...");
        success = false;
        return;
    }
    
    std::vector<int> robots = getAvailableRobots();
    Vector2 ballPos = Vector2(world.ball.pos);
    auto penaltyTakerID = get_robot_closest_to_point(robots, world, ballPos);

    if (!penaltyTakerID) {
        ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "No robot available to take the penalty!");
        success = false;
    	return;
    }

    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Robot chosen for penalty: " << *penaltyTakerID);

    // Get the default roledirective publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	ROS_DEBUG_NAMED(ROS_LOG_NAME, "Initializing Jim_PenaltyPlay...");


    // =============================
    // Initialize the Ball Getter
    // =============================
    {
        roboteam_msgs::RoleDirective rd;
        rd.robot_id = *penaltyTakerID;
        bt::Blackboard bb;
        claim_robot(*penaltyTakerID);
        activeRobot = *penaltyTakerID;

        bb.SetInt("ROBOT_ID", *penaltyTakerID);
        bb.SetInt("KEEPER_ID", RobotDealer::get_keeper());

        bb.SetBool("GetBall_A_passToBestAttacker", true); 
        bb.SetBool("ShootAtGoalV2_A_waitForDO_PENALTY", true);

        // Create message
        rd.tree = "rtt_jim/ShootAtGoalRole";
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

void Jim_PenaltyPlay::initShootout(){
    // Get the world
    const roboteam_msgs::World& world = LastWorld::get();
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();


    // Check if there is a bot on the field
    if(world.us.size() == 0){
        ROS_ERROR_STREAM_NAMED(ROS_LOG_NAME, "No robots on the field for the shootout!");
        return;
    }

    // Get all of our bots on the field
    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, world.us.size() << " robots on the field. Assigning closest one as attacker, moving others to the back");

    float closestDistance = 9999;
    int closestID = world.us.at(0).id;
    // Get robot closest to ball
    for(const roboteam_msgs::WorldRobot& bot : world.us){
        float distance = ((Vector2)bot.pos - (Vector2)world.ball.pos).length();
        if(distance < closestDistance){
            closestDistance = distance;
            closestID = bot.id;
        }
    }
    ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Robot closest to ball : " << closestID << " with " << closestDistance << "m");

    for(const roboteam_msgs::WorldRobot& bot : world.us){

        int robotID = bot.id;

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = robotID;

        // Create blackboard
        bt::Blackboard bb;
//        claim_robot(robotID);
//        activeRobot = *penaltyTakerID;

        bb.SetInt("ROBOT_ID", robotID);
        bb.SetInt("KEEPER_ID", -1);

        if(robotID == closestID) {
            // Set kicker
//            bb.SetBool("GetBall_A_passToBestAttacker", true);
//            bb.SetBool("ShootAtGoalV2_A_waitForDO_PENALTY", true);
//            rd.tree = "rtt_jim/ShootAtGoalRole";
            bb.SetBool("GetBall_A_passOn", true);
            bb.SetString("GetBall_A_aimAt", "theirgoal");
            rd.tree = "rtt_jim/GetBallRole";
            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Claiming robot " << robotID << " for the kicker");
        }else{
            // Move other robots out of the way

            ScopedBB(bb, "GoToPos_A")
                .setDouble("angleGoal", 0)
                .setDouble("xGoal"    , bot.pos.x)
                .setDouble("yGoal"    , bot.pos.y)
                .setBool("avoidRobots", true)
                .setBool("avoidBall"  , true)
                .setBool("stayAwayFromBall"  , true)
                .setBool("enterDefenseAreas"  , true)
                .setDouble("maxSpeed", 2.0)
            ;

            rd.tree = "rtt_emiel/GoToPos_Nonstop_Role";
            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Moving robot " << robotID << " to our goal line");
        }

        // Create message
        rd.blackboard = bb.toMsg();

        // Add random token and save it for later
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        rd.token = unique_id::toMsg(token);

        if(robotID == closestID){
            kickerToken = token;
        }

        // Send to rolenode
        pub.publish(rd);

    }
}

bt::Node::Status Jim_PenaltyPlay::Update() {
    if (!success) {
    	return Status::Failure;
    }

    if(!inShootout) {
        for (auto token : tokens) {
            if (feedbacks.find(token) != feedbacks.end()) {
                Status status = feedbacks.at(token);
                if (status == Status::Success) {
                    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Success!");
                }
                if (status == Status::Failure) {
                    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Failure!");
                }
                return status;
            }
        }
    }else{
        if(!kickerToken)
            return Status::Running;

        if (feedbacks.find(*kickerToken) != feedbacks.end()) {
            ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Kicker finished! Again..");
            Jim_PenaltyPlay::initShootout();

        }

        return Status::Running;
    }

    return Status::Running;
}

} // rtt
