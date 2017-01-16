#include "ros/ros.h"

#include "roboteam_tactics/utils/ComputePassPoint.h"

#include "roboteam_tactics/utils/LastWorld.h"

#include "roboteam_tactics/utils/Draw.h"
#include "roboteam_utils/Vector2.h"
// #include "roboteam_utils/constants.h"

bool worldCallBack = 0;

void msgCallBack(const roboteam_msgs::WorldConstPtr& world, rtt::PassPoint* passPoint) {
	rtt::LastWorld::set(*world);
	roboteam_utils::Vector2 testPosition(-3.0, 0.0);
	
	roboteam_msgs::World newworld = rtt::LastWorld::get();
	double viewOfGoal = passPoint->calcViewOfGoal(testPosition, newworld);
	ROS_INFO_STREAM("viewOfGoal: " << viewOfGoal);
	if (!worldCallBack) worldCallBack = true;
}

void msgCallbackFieldGeometry(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    rtt::LastWorld::set_field(geometry->field);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ComputePassPointTest");
	ros::NodeHandle n;

	rtt::PassPoint passPoint;

	ros::Subscriber sub = n.subscribe<roboteam_msgs::World> ("world_state", 1000, boost::bind(&msgCallBack, _1, &passPoint));
	ros::Subscriber geom_sub = n.subscribe<roboteam_msgs::GeometryData> ("vision_geometry", 1000, msgCallbackFieldGeometry);
	
	while (!worldCallBack) {
		ROS_INFO_STREAM("waiting for world message!");
		ros::spinOnce();
	}

	// rtt::Draw drawer;

	// rtt::Cone cone(rtt::Vector2(0, 0), rtt::Vector2(2, 0), 0.5);
	// drawer.DrawLine("line1", cone.start, cone.side1);
	// drawer.DrawLine("line2", cone.start, cone.side2);

	// rtt::Cone cone2(roboteam_utils::Vector2(0, 0), roboteam_utils::Vector2(2, -1), 0.4);
	// drawer.DrawLine("2line1", cone2.start, cone2.side1);
	// drawer.DrawLine("2line2", cone2.start, cone2.side2);
	

	// roboteam_utils::Vector2 testPoint1(1.0, 0.1);
	// roboteam_utils::Vector2 testPoint2(1.0, 0.2);
	// roboteam_utils::Vector2 testPoint3(1.0, 0.24);

	// drawer.DrawPoint("point1", testPoint1);
	// drawer.DrawPoint("point2", testPoint2);
	// drawer.DrawPoint("point3", testPoint3);

	// if (cone.IsWithinCone(testPoint1)) {
	// 	ROS_INFO_STREAM("point1 within cone");
	// }
	// if (cone.IsWithinCone(testPoint2)) {
	// 	ROS_INFO_STREAM("point2 within cone");
	// }
	// if (cone.IsWithinCone(testPoint3)) {
	// 	ROS_INFO_STREAM("point3 within cone");
	// }

	// if (cone.DoConesOverlap(cone2)) {
	// 	ROS_INFO_STREAM("overlap!");
	// }

	// roboteam_utils::Vector2 testPoint1(-3.0, 1.0);
	// roboteam_utils::Vector2 testPoint2(-3.0, -1.0);
	// double score1 = rtt::computePassPointScore(testPoint1);
	// double score2 = rtt::computePassPointScore(testPoint2);

	ROS_INFO_STREAM("received world message!");

	
	
	
	
	// double bestScore = passPoint.computeBestPassPoint();

	while (ros::ok()) {
		// roboteam_msgs::World world = rtt::LastWorld::get();
		
		// ROS_INFO_STREAM("viewOfGoal: " << viewOfGoal);
		ros::spinOnce();
	}
	

	// ROS_INFO_STREAM("score1: " << score1 << " score2: " << score2);

	return 0;
}