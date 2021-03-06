#include <array>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_tactics/generated/alltrees.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/conditions/IsBallInGoal.h"

namespace rtt {

bool success1;
bool failure1;
bool success2;
bool failure2;
bool successAll;
bool receivedTheFirstWorldMessage;
int primaryAttacker;
int secondaryAttacker;


void worldStateCallbackInit(const roboteam_msgs::WorldConstPtr& world) {
    rtt::LastWorld::set(*world);
    receivedTheFirstWorldMessage = true;
}

// void worldStateCallback(const roboteam_msgs::WorldConstPtr& world, bt::BehaviorTree* tree1, bt::Blackboard::Ptr bb1, bt::BehaviorTree* tree2, bt::Blackboard::Ptr bb2) {
//     rtt::LastWorld::set(*world);

//     roboteam_msgs::World getworld = rtt::LastWorld::get();
//     roboteam_msgs::WorldBall ball = getworld.ball;
//     Vector2 center = Vector2(ball.pos.x, ball.pos.y);

//     Vector2 secondaryAttackerPos(getworld.us.at(secondaryAttacker).pos.x, getworld.us.at(secondaryAttacker).pos.y);
//     bb2->SetDouble("GetBall_A_getBallAtX", secondaryAttackerPos.x);
//     bb2->SetDouble("GetBall_A_getBallAtY", secondaryAttackerPos.y);
    
//     if (!success1 && !failure1) {
//         bt::Node::Status status1 = tree1->Update();
//         if (status1 == bt::Node::Status::Success) {success1 = true;}
//         if (status1 == bt::Node::Status::Failure) {failure1 = true;}
//     }

//     if (!success2 && !failure2) {
//         bt::Node::Status status2 = tree2->Update();
//         if (status2 == bt::Node::Status::Success) {success2 = true;}
//         if (status2 == bt::Node::Status::Failure) {failure2 = true;}
//     }

//     auto bb3 = std::make_shared<bt::Blackboard>();
//     bb3->SetBool("our_goal", false);
//     IsBallInGoal isBallInGoal("", bb3);
//     bt::Node::Status weScored = isBallInGoal.Update();
//     if (weScored == bt::Node::Status::Success) {
//         successAll = true;
//     } else {
//         successAll = false;
//     }
// }

void worldStateCallback2(const roboteam_msgs::WorldConstPtr& world) {
    rtt::LastWorld::set(*world);
}

void fieldUpdateCallback(const roboteam_msgs::GeometryDataConstPtr& geom) {
    rtt::LastWorld::set_field(geom->field);
}

void initializeRoles(bt::Blackboard::Ptr bb1, bt::Blackboard::Ptr bb2) {
    roboteam_msgs::World world = rtt::LastWorld::get();
    Vector2 ballPos(world.ball.pos.x, world.ball.pos.y);

    // This tactic directs two robots
    std::array<int, 2> availableRobots = {0, 1};
    auto bot0 = getWorldBot(0);
    auto bot1 = getWorldBot(1);
    Vector2 firstRobotPos(bot0->pos);
    Vector2 secondRobotPos(bot1->pos);
    

    // Assign the role of primary attacker to the robot that is closest to the ball
    if ((firstRobotPos - ballPos).length() < (secondRobotPos - ballPos).length()) {
        primaryAttacker = availableRobots.at(0);
        secondaryAttacker = availableRobots.at(1);
    } else {
        primaryAttacker = availableRobots.at(1);
        secondaryAttacker = availableRobots.at(0);
    }


    ROS_INFO_STREAM("primaryAttacker: " << primaryAttacker << " secondaryAttacker: " << secondaryAttacker);

    // Attacker 1
    bb1->SetInt("ROBOT_ID", primaryAttacker);

    // Get the ball!
    bb1->SetBool("GetBall_A_intercept", false);
    
    // If you can see the goal, aim towards it
    bb1->SetBool("AimAt_A_setRosParam", false);
    bb1->SetString("AimAt_A_At", "theirgoal");

    // Else, if you can see the other attacker, aim to him
    bb1->SetBool("CanSeeRobot_A_our_team", true);
    bb1->SetInt("CanSeeRobot_A_targetID", secondaryAttacker);
    bb1->SetBool("AimAt_B_setRosParam", true);
    bb1->SetString("AimAt_B_At", "robot");
    bb1->SetInt("AimAt_B_AtRobot", secondaryAttacker);

    // And then shoot!


    // Attacker 2
    bb2->SetInt("ROBOT_ID", secondaryAttacker);

    // Make sure you stand free to receive the ball
    bb2->SetInt("StandFree_A_theirID", primaryAttacker);
    bb2->SetBool("StandFree_A_setRosParam", true);
    bb2->SetBool("StandFree_A_ourTeam", true);
    bb2->SetDouble("StandFree_A_distanceFromPoint", 0.3);

    // Receive the ball
    bb2->SetBool("GetBall_A_intercept", true);
    // bb2->SetDouble("GetBall_A_getBallAtX", 0.0); // these positions will be updated in the world callback to match the robot's current position
    // bb2->SetDouble("GetBall_A_getBallAtY", 0.0);
    bb2->SetBool("GetBall_A_getBallAtCurrentPosition", true);

    // Aim at goal
    bb2->SetBool("AimAt_A_setRosParam", false);
    bb2->SetString("AimAt_A_At", "theirgoal");

    // And shoot!
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "AttackTactic");
    ros::NodeHandle n;

    ros::Subscriber subInit = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, worldStateCallbackInit);
    while (!receivedTheFirstWorldMessage) {
        ROS_INFO("Waiting for the first world message to initialize the tactic");
        ros::spinOnce();
    }
    subInit.shutdown();

    auto role1 = make_CoolTree();
    auto bb1 = role1.GetBlackboard();
    auto role2 = make_SuperCoolTree();
    auto bb2 = role2.GetBlackboard();
    initializeRoles(bb1, bb2);
    
    // ros::Subscriber sub = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, boost::bind(&worldStateCallback, _1, &role1, bb1, &role2, bb2));
    ros::Subscriber sub = n.subscribe<roboteam_msgs::World> (TOPIC_WOLRD_STATE, 1000, worldStateCallback2);
    ros::Subscriber subField = n.subscribe(TOPIC_GEOMETRY, 10, &fieldUpdateCallback);

    ros::Rate rate(60);
    bool finished1;
    bool finished2;

    while (ros::ok()) {
        ros::spinOnce();
        if (!finished1) {
            if (role1.Update() != bt::Node::Status::Running) {
                finished1 = true;
            }
        }
        if (!finished2) {
            if (role2.Update() != bt::Node::Status::Running) {
                finished2 = true;
            }
        }
        if (finished1 && finished2) {break;}
        // rate.sleep();
    }


    // while(ros::ok()) {
    //     ros::spinOnce();
    //     bool dontStopMeNow_ImHavingSuchAGoodTime = false;
    //     if (successAll) {
    //         if (dontStopMeNow_ImHavingSuchAGoodTime) {
    //             ROS_INFO("start again");
    //             success1 = false;
    //             success2 = false;
    //             successAll = false;
    //             initializeRoles(bb1, bb2);
    //         } else {
    //             ROS_INFO("finished");
    //             break;
    //         }
    //     }
    //     if (failure1 || failure2) {
    //         ROS_INFO("failed :(");
    //         break;
    //     }
    // }
    return 0;
}

} // rtt

int main(int argc, char **argv) {
    return rtt::main(argc, argv);
}
