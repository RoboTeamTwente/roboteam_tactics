#include <QtNetwork>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <string>

#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_msgs/GeometryData.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/utils/OpportunityFinder.h"
#include "roboteam_tactics/tactics/PassToTactic.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/conditions/IsBallInGoal.h"

#define RTT_CURRENT_DEBUG_TAG Learner

bool firstWorldCallBack = false;
bool firstFieldCallBack = false;
QUdpSocket udpsocket;

// Place a robot in a specified position and orientation
void placeRobot(uint id, bool yellow_team, rtt::Vector2 pos, double dir) {
	grSim_Packet packet;

	grSim_Replacement* replace = packet.mutable_replacement();
	grSim_RobotReplacement* robot = replace->add_robots();

	robot->set_x(pos.x);
	robot->set_y(pos.y);
	robot->set_dir(dir);
	robot->set_id(id);
	robot->set_yellowteam(yellow_team);

	QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());

    QUdpSocket udpsocket;

    // Send to IP address and port specified in grSim
    std::string grsim_ip = "127.0.0.1";
    int grsim_port = 20011;
    ros::param::get("grsim/ip", grsim_ip);
    ros::param::get("grsim/port", grsim_port);
    udpsocket.writeDatagram(dgram, QHostAddress(QString::fromStdString(grsim_ip)), grsim_port);
}


// Place the ball in a specified position
void placeBall(rtt::Vector2 pos) {
    grSim_Packet packet;

    grSim_Replacement* replace = packet.mutable_replacement();
    grSim_BallReplacement* ball = replace->mutable_ball();

    ball->set_x(pos.x);
    ball->set_y(pos.y);
    ball->set_vx(0);
    ball->set_vy(0);

    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());

    QUdpSocket udpsocket;

    // Send to IP address and port specified in grSim
    std::string grsim_ip = "127.0.0.1";
    int grsim_port = 20011;
    ros::param::get("grsim/ip", grsim_ip);
    ros::param::get("grsim/port", grsim_port);
    udpsocket.writeDatagram(dgram, QHostAddress(QString::fromStdString(grsim_ip)), grsim_port);
}

// Callback function for the feedback coming from the role nodes. This feedback is used in the tactic file (PassToTactic.cpp)
void feedbackCallback(const roboteam_msgs::RoleFeedbackConstPtr &msg) {
    auto uuid = unique_id::fromMsg(msg->token);
    if (msg->status == roboteam_msgs::RoleFeedback::STATUS_FAILURE) {
        rtt::feedbacks[uuid] = bt::Node::Status::Failure;
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_INVALID) {
        rtt::feedbacks[uuid] = bt::Node::Status::Invalid;
    } else if (msg->status == roboteam_msgs::RoleFeedback::STATUS_SUCCESS) {
        rtt::feedbacks[uuid] = bt::Node::Status::Success;
    }
}

void worldCallback(const roboteam_msgs::WorldConstPtr& world) {
	rtt::LastWorld::set(*world);
	if (!firstWorldCallBack) firstWorldCallBack = true;
}

void fieldCallback(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    rtt::LastWorld::set_field(geometry->field);
    if (!firstFieldCallBack) firstFieldCallBack = true;
}

double evaluatePass(rtt::time_point start, uint epoch, bool succeeded, bool scored) {
	rtt::time_point finish = rtt::now();
	auto duration = rtt::time_difference_milliseconds(start, finish);
    
	if (succeeded) {
		if (scored) {
			RTT_DEBUG("Epoch %d succeeded, and we scored after %ld ms! \n", epoch, duration.count());
		} else {
			RTT_DEBUG("Epoch %d succeeded, but we didn't score :(", epoch);
		}
	} else {
		RTT_DEBUG("Epoch %d failed after %ld ms :( \n", epoch, duration.count());
	}
	
    roboteam_msgs::World world = rtt::LastWorld::get();
	return 0.0;
}

bool didWeScore() {
	auto bb = std::make_shared<bt::Blackboard>();
	bb->SetBool("our_goal", false);
	rtt::IsBallInGoal isBallInGoal("", bb);

	if (isBallInGoal.Update() == bt::Node::Status::Success) {
		return true;
	} else {
		return false;
	}
}


int main(int argc, char **argv) {
	RTT_DEBUG("Initializing Learner \n");

	ros::init(argc, argv, "Learner");
	ros::NodeHandle n;
	ros::Rate rate(60);	

	std::cout << "Please enter the name of the weights file you want to use: \n";
	std::string fileName;
	std::cin >> fileName;
	std::cout << "Using path: " << fileName << " \n";

	std::cout << "Please enter the robotID for which you want to calculate a point: \n";
	int robotID;
	std::cin >> robotID;
	std::cout << "Using robotID: " << robotID << " \n";

	std::string target = "theirgoal";
	int targetID = 2;

	// OpportunityFinder, for the functions for calculating the best pass point and adapting the weights
	rtt::OpportunityFinder opportunityFinder;
	opportunityFinder.Initialize(fileName, robotID, target, targetID);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, &worldCallback);
	ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, fieldCallback);	
	// Waiting for the first world callback
	firstWorldCallBack = false;
	while (!firstWorldCallBack) {
		ros::spinOnce();
	}

	firstFieldCallBack = false;
	while (!firstFieldCallBack) {
		ros::spinOnce();
	}

	rtt::time_point start = rtt::now();
	
	// Benchmark!!
	// int count = 0;
	// int nIterations = 100;
	// std::vector<int> times;
	// while (count < nIterations) {
	// 	opportunityFinder.Initialize(fileName);
	// 	opportunityFinder.computeBestOpportunity(ROBOT_ID, target, targetID);
	// 	ros::spinOnce();
	// 	int timePast = rtt::time_difference_milliseconds(start, rtt::now()).count();
	// 	times.push_back(timePast);
	// 	count++;
	// 	start = rtt::now();
	// }
	// int sumOfTimes = std::accumulate(times.begin(), times.end(), 0);	
	// double avg = sumOfTimes / nIterations;
	// RTT_DEBUG("Average time per iteration: %f \n", avg);
	// return 0;

	while (ros::ok()) {
		if (rtt::time_difference_milliseconds(start, rtt::now()).count() > 500) {
			opportunityFinder.Initialize(fileName, robotID, target, targetID);
			opportunityFinder.computeBestOpportunity();
			ros::spinOnce();
			start = rtt::now();
		}
	}
	
	return 0;

	// PassToTactic, for executing the tactic that controls two robots to test the pass
	// auto bb = std::make_shared<bt::Blackboard>();
	// rtt::PassToTactic passToTactic("", bb);	

	// // Subscribers
	// ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, fieldCallback);	
	// ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 0, &feedbackCallback);
	

	

	// // Choose a point to test

	// rtt::time_point start_time;
	// rtt::time_point finish_time;
	// rtt::time_point succes_time;
	// uint epoch = 0;

	// // Some control booleans
	// bool tacticSucces = true;
	// bool tacticFailure = false;
	// // bool justFinishedTactic = false;
	// bool scored = false;
	// bool waitForGoal = false;
	
	// // Main program loop
	// while (ros::ok()) {

	// 	if (!tacticSucces && !tacticFailure) {
	// 		bt::Node::Status status = passToTactic.Update();

	// 		// If the tactic succeeds or fails, change some control booleans so we take the correct next step, otherwise just keep spinnin'
	// 		if (status == bt::Node::Status::Success) {
	// 			tacticSucces = true;
	// 			succes_time = rtt::now();
	// 			waitForGoal = true; 
	// 		}
	// 		if (status == bt::Node::Status::Failure) {
	// 			tacticFailure = true;
	// 			// finish_time = rtt::now();
	// 		}

	// 	} else {

	// 		// Because the tactic succeeded we´d like to wait for a while to see if it results in a goal, but not too long in case it doesn't (max 5 seconds)
	// 		if (waitForGoal) {
	// 			if (!scored && (rtt::time_difference_milliseconds(rtt::now(), succes_time).count() < 5000)) {
	// 				if (didWeScore()) {
	// 					scored = true;
	// 					waitForGoal = false;
	// 				}
	// 			}

	// 		} else {

	// 			// This means we're done, now we can evaluate what happened this epoch, for this we look at the time it took to complete the epoch, as well
	// 			// as whether the tactic succeeded, and whether it resulted in a goal
	// 			bool succeeded;
	// 			if (tacticSucces) {succeeded = true;}
	// 			if (tacticFailure) {succeeded = false;}

	// 			if (epoch >= 1) {
	// 				evaluatePass(start_time, epoch, succeeded, scored);
	// 			}

	// 			// Reset the field and the control booleans and start a new epoch
	// 			placeBall(rtt::Vector2(0.0, 0.0));
	// 			placeRobot(1, true, rtt::Vector2(0.5, 0.0), M_PI);
	// 			placeRobot(2, true, rtt::Vector2(-3.0, 1.0), M_PI);
	// 			passToTactic.Initialize(); // arg: passTo

	// 			tacticSucces = false;
	// 			tacticFailure = false;
	// 			scored = false;

	// 			epoch++;
	// 			start_time = rtt::now();
	// 		}
	// 	}

	// 	// Spin ROS at the specified rate
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }
		
	// return 0;
}