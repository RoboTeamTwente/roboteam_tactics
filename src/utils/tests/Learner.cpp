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

#include <cmath>
#include <iostream>

#define RTT_CURRENT_DEBUG_TAG Learner

bool firstWorldCallBack = false;
bool tacticSucces = true;
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

void worldCallback(const roboteam_msgs::WorldConstPtr& world, rtt::PassPoint* passPoint, rtt::PassToTactic* passToTactic) {
	rtt::LastWorld::set(*world);
	bt::Node::Status tacticStatus = passToTactic->Update();
	if (tacticStatus == bt::Node::Status::Success) {
		tacticSucces = true;
	}
	if (tacticStatus == bt::Node::Status::Running) {
	}
	if (!firstWorldCallBack) firstWorldCallBack = true;
}

void fieldCallback(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    rtt::LastWorld::set_field(geometry->field);
}

double evaluatePass(rtt::time_point start, rtt::time_point finish) {
	auto duration = rtt::time_difference_milliseconds(start, finish);
    RTT_DEBUG("Tactic took %ld ms to execute \n", duration.count());
    // std::cout << duration.count();
    // ROS_INFO_STREAM("duration " << duration.count());
	return 0.0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Learner");
	ros::NodeHandle n;

	rtt::PassPoint passPoint;

	auto bb = std::make_shared<bt::Blackboard>();
	rtt::PassToTactic passToTactic("", bb);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldCallback, _1, &passPoint, &passToTactic));
	ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, fieldCallback);	
	ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 0, &feedbackCallback);

	roboteam_utils::Vector2 passTo(2.0, 0.0);
	placeBall(roboteam_utils::Vector2(0.0, 0.0));	
	// double bestScore = passPoint.computeBestPassPoint();

	rtt::time_point start = rtt::now();
	
	while (!firstWorldCallBack) {
		ros::spinOnce();
	}

	while (ros::ok()) {
		ros::spinOnce();


		if (tacticSucces) {
			rtt::time_point finish = rtt::now();
			double a = evaluatePass(start, finish);

			// RTT_DEBUG("Cycle completed, starting next\n");
			placeBall(roboteam_utils::Vector2(0.0, 0.0));
			placeRobot(1, true, roboteam_utils::Vector2(0.5, 0.0), M_PI);
			placeRobot(2, true, roboteam_utils::Vector2(-3.0, 1.0), M_PI);

			// Wait for the world to realize that positions have changed...
			firstWorldCallBack = false;
			while (!firstWorldCallBack) {
				ros::spinOnce();
			}

			// Choose a point to test
			roboteam_utils::Vector2 passTo = passPoint.computeBestPassPoint();
			
			// Initialize the tactic with the chosen passPoint
			start = rtt::now();
			passToTactic.Initialize(passTo);
			tacticSucces = false;
		}
	}
	return 0;
}