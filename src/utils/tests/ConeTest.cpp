#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/Cone.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"


int main() {
	roboteam_utils::Vector2 startPoint = roboteam_utils::Vector2(0.0, 0.0);
	roboteam_utils::Vector2 centerPoint = roboteam_utils::Vector2(2.0, 0.0);
	double distance = 0.5;
	rtt::Cone cone(startPoint, centerPoint, distance);

	// roboteam_utils::Vector2 startPoint2 = roboteam_utils::Vector2(0.0, 0.0);
	// roboteam_utils::Vector2 centerPoint2 = roboteam_utils::Vector2(2.0, 1.0);
	// double distance2 = 0.5;
	// rtt::Cone cone2(startPoint2, centerPoint2, distance2);

	// bool check = cone.DoConesOverlap(cone2);
	// ROS_INFO_STREAM("do cones overlap: " << check);

	// rtt::Cone cone3 = cone.MergeCones(cone2);
	// ROS_INFO_STREAM("merged");

	// ROS_INFO_STREAM("cone1 center:" << cone.center.x << " " << cone.center.y << " distance: " << cone.radius);
	// ROS_INFO_STREAM("cone1 center:" << cone2.center.x << " " << cone2.center.y << " distance: " << cone2.radius);
	// ROS_INFO_STREAM("cone1 center:" << cone3.center.x << " " << cone3.center.y << " distance: " << cone3.radius);

	// roboteam_utils::Vector2 cone1Side1 = (cone.center-cone.start).rotate(cone.angle);
	// cone1Side1 = cone1Side1.scale(1/cone1Side1.length());
	// roboteam_utils::Vector2 cone1Side2 = (cone.center-cone.start).rotate(-cone.angle);
	// cone1Side2 = cone1Side2.scale(1/cone1Side2.length());
	// roboteam_utils::Vector2 cone2Side1 = (cone2.center-cone2.start).rotate(cone2.angle);
	// cone2Side1 = cone2Side1.scale(1/cone2Side1.length());
	// roboteam_utils::Vector2 cone2Side2 = (cone2.center-cone2.start).rotate(-cone2.angle);
	// cone2Side2 = cone2Side2.scale(1/cone2Side2.length());
	// roboteam_utils::Vector2 cone3Side1 = (cone3.center-cone3.start).rotate(cone3.angle);
	// cone3Side1 = cone3Side1.scale(1/cone3Side1.length());
	// roboteam_utils::Vector2 cone3Side2 = (cone3.center-cone3.start).rotate(-cone3.angle);
	// cone3Side2 = cone3Side2.scale(1/cone3Side2.length());

	// ROS_INFO_STREAM("cone1Side1:" << cone1Side1.x << " " << cone1Side1.y);
	// ROS_INFO_STREAM("cone1Side2:" << cone1Side2.x << " " << cone1Side2.y);
	// ROS_INFO_STREAM("cone2Side1:" << cone2Side1.x << " " << cone2Side1.y);
	// ROS_INFO_STREAM("cone2Side2:" << cone2Side2.x << " " << cone2Side2.y);
	// ROS_INFO_STREAM("cone3Side1:" << cone3Side1.x << " " << cone3Side1.y);
	// ROS_INFO_STREAM("cone3Side2:" << cone3Side2.x << " " << cone3Side2.y);
	return 0;
}
