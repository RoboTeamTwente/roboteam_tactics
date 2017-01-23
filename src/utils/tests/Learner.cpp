#include "ros/ros.h"
#include "roboteam_tactics/utils/ComputePassPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/Draw.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/tactics/PassToTactic.h"
#include "roboteam_tactics/utils/utils.h"

#include <QtNetwork>
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"
#include "roboteam_tactics/utils/FeedbackCollector.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/conditions/IsBallInGoal.h"

#include <cmath>
#include <iostream>

#define RTT_CURRENT_DEBUG_TAG Learner

bool firstWorldCallBack = false;
bool tacticSucces = true;
bool tacticFailure = false;
QUdpSocket udpsocket;

// Place a robot in a specified position and orientation
void placeRobot(uint id, bool yellow_team, roboteam_utils::Vector2 pos, double dir) {
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
void placeBall(roboteam_utils::Vector2 pos) {
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
}

double evaluatePass(rtt::time_point start, uint epoch, bool succeeded, bool scored) {
	rtt::time_point finish = rtt::now();
	auto duration = rtt::time_difference_milliseconds(start, finish);
    
	if (succeeded) {
		RTT_DEBUG("Epoch %d succeeded in %ld ms \n", epoch, duration.count());
	} else {
		RTT_DEBUG("Epoch %d failed after %ld ms \n", epoch, duration.count());
	}
	
    roboteam_msgs::World world = rtt::LastWorld::get();
	return 0.0;
}


int main(int argc, char **argv) {
	RTT_DEBUG("Initializing Learner \n");
	ros::init(argc, argv, "Learner");
	ros::NodeHandle n;

	RTT_DEBUGLN("Starting the tactic");
	ros::Rate rate(60);	

	rtt::PassPoint passPoint;

	RTT_DEBUGLN("Initializing new passToTactic");
	auto bb = std::make_shared<bt::Blackboard>();
	rtt::PassToTactic passToTactic("", bb);	

	ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, fieldCallback);	
	ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 0, &feedbackCallback);
	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, &worldCallback);

	// Waiting for the first world callback
	firstWorldCallBack = false;
	while (!firstWorldCallBack) {
		ros::spinOnce();
	}

	// Choose a point to test
	roboteam_utils::Vector2 passTo = passPoint.computeBestPassPoint();
	// passToTactic.Initialize(passTo);

	
	while (ros::ok()) {

		if (!tacticSucces && !tacticFailure) {
			bt::Node::Status status = passToTactic.Update();	
			if (status == bt::Node::Status::Success) {
				RTT_DEBUGLN("Succes!");
				tacticSucces = true;
			}
			if (status == bt::Node::Status::Failure) {
				RTT_DEBUGLN("Failure!");
				tacticFailure = true;
			}
		} else {
			placeBall(roboteam_utils::Vector2(0.0, 0.0));
			placeRobot(1, true, roboteam_utils::Vector2(0.5, 0.0), M_PI);
			placeRobot(2, true, roboteam_utils::Vector2(-3.0, 1.0), M_PI);
			passToTactic.Initialize(passTo);

			tacticSucces = false;
			tacticFailure = false;
		}
		ros::spinOnce();
		rate.sleep();
	}
		
	return 0;
}