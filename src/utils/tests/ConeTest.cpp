#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
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

	roboteam_utils::Vector2 startPoint2 = roboteam_utils::Vector2(0.0, 0.0);
	roboteam_utils::Vector2 centerPoint2 = roboteam_utils::Vector2(2.0, 1.0);
	double distance2 = 0.5;
	rtt::Cone cone2(startPoint2, centerPoint2, distance2);

	bool check = cone.DoConesOverlap(cone2);
	ROS_INFO_STREAM("do cones overlap: " << check);

	rtt::Cone cone3 = cone.MergeCones(cone2);
	ROS_INFO_STREAM("merged");

	ROS_INFO_STREAM("cone1 center:" << cone.center.x << " " << cone.center.y << " distance: " << cone.radius);
	ROS_INFO_STREAM("cone1 center:" << cone2.center.x << " " << cone2.center.y << " distance: " << cone2.radius);
	ROS_INFO_STREAM("cone1 center:" << cone3.center.x << " " << cone3.center.y << " distance: " << cone3.radius);



	// roboteam_utils::Vector2 isInConeTest1 = roboteam_utils::Vector2(4.0, 0.1);
	// roboteam_utils::Vector2 isInConeTest2 = roboteam_utils::Vector2(3.0, 0.2);
	// roboteam_utils::Vector2 isInConeTest3 = roboteam_utils::Vector2(1.0, -0.01);
	// ROS_INFO_STREAM("test 1: " << cone.IsWithinCone(isInConeTest1));
	// ROS_INFO_STREAM("test 2: " << cone.IsWithinCone(isInConeTest2));
	// ROS_INFO_STREAM("test 3: " << cone.IsWithinCone(isInConeTest3));

	// ROS_INFO_STREAM("pointtest1: " << cone.ClosestPointOnSide(isInConeTest1).x << " " << cone.ClosestPointOnSide(isInConeTest1).y);
	// ROS_INFO_STREAM("pointtest2: " << cone.ClosestPointOnSide(isInConeTest3).x << " " << cone.ClosestPointOnSide(isInConeTest3).y);
	return 0;
}
