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

bool worldCallBack = false;
bool tacticSucces = false;
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
	if (!worldCallBack) worldCallBack = true;
}

void fieldCallback(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    rtt::LastWorld::set_field(geometry->field);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ComputePassPointTrain");
	ros::NodeHandle n;

	rtt::PassPoint passPoint;

	auto bb = std::make_shared<bt::Blackboard>();
	rtt::PassToTactic passToTactic("", bb);

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&worldCallback, _1, &passPoint, &passToTactic));
	ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, fieldCallback);	
	ros::Subscriber feedbackSub = n.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 10, &feedbackCallback);

	roboteam_utils::Vector2 passTo(2.0, 0.0);
	placeBall(roboteam_utils::Vector2(0.0, 0.0));	
	// double bestScore = passPoint.computeBestPassPoint();
	
	while (!worldCallBack) {
		ros::spinOnce();
	}

	passToTactic.Initialize(passTo);

	while (ros::ok()) {
		ros::spinOnce();

		if (tacticSucces) {

			ROS_INFO_STREAM("ComputePassPointTest done, shutting down");
			// placeBall(roboteam_utils::Vector2(0.0, 0.0));
			// passToTactic.Initialize(passTo);
			// tacticSucces = false;
			return 0;
		}
	}
	return 0;
}