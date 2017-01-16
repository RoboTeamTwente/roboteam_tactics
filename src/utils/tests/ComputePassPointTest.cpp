#include "ros/ros.h"
#include "roboteam_tactics/utils/ComputePassPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/Draw.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/tactics/PassToTactic.h"

#include <QtNetwork>
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"

bool worldCallBack = 0;
QUdpSocket udpsocket;

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


void worldCallback(const roboteam_msgs::WorldConstPtr& world, rtt::PassPoint* passPoint) {
	rtt::LastWorld::set(*world);
	// roboteam_utils::Vector2 testPosition(-3.0, 0.0);
	
	// roboteam_msgs::World newworld = rtt::LastWorld::get();
	// double viewOfGoal = passPoint->calcViewOfGoal(testPosition, newworld);
	// ROS_INFO_STREAM("viewOfGoal: " << viewOfGoal);
	if (!worldCallBack) worldCallBack = true;
}

void fieldCallback(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    rtt::LastWorld::set_field(geometry->field);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ComputePassPointTest");
	ros::NodeHandle n;

	rtt::PassPoint passPoint;
	rtt::PassToTactic passToPoint;

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldCallback, _1, &passPoint));
	ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, fieldCallback);
	
	while (!worldCallBack) {
		ros::spinOnce();
	}
	
	double bestScore = passPoint.computeBestPassPoint();
	roboteam_utils::Vector2 passTo(2.0, 0.0);

	while (ros::ok()) {
		ros::spinOnce();
	}
	
	return 0;
}