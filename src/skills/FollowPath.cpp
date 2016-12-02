#include <vector>

#include "ros/ros.h"


#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/navsim.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/FollowPath.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/ResultCodes.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

FollowPath::FollowPath(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {
		// state = "computePath";
        state = COMPUTE;
        client = n.serviceClient<roboteam_msgs::navsim>("navsim");
}

std::vector<roboteam_msgs::Vector2f> FollowPath::ComputePath(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 goalPos) {
	char result = RESULT_VELOCITY_DISREGARD;

    // We need to calculate a new path, so contact NavSim
    roboteam_msgs::navsim srv;
    srv.request.start.x = robotPos.x;
    srv.request.start.y = robotPos.y;
    srv.request.goal.x = goalPos.x;
    srv.request.goal.y = goalPos.y;

    if (client.call(srv)) {
        // The call succeeded, but that does not mean a path was actually found.
        result = srv.response.flags;
        // printf("%s\n", describe_flags(result).c_str());
        if (~result & RESULT_FAIL) {
            // A path was found, so set it.
            // points = srv.response.path;
        } else {
        	ROS_INFO("No path found :(");
        }
    } else {
        ROS_INFO("roboteam_nav call failed :(");
        // result |= RESULT_FAIL | RESULT_FAIL_UNKOWN;
    }

	return points;
}

void FollowPath::CallGoToPos(roboteam_msgs::Vector2f point, double wGoal, int robotID) {
	auto bb = std::make_shared<bt::Blackboard>();
    bb->SetDouble("xGoal", point.x);
    bb->SetDouble("yGoal", point.y);
    bb->SetDouble("wGoal", wGoal);
    bb->SetInt("ROBOT_ID", robotID);

    if (points.size() > 1) {
    	bb->SetBool("endPoint", false);
    } else {
    	bb->SetBool("endPoint", true);
    }
    if (goToPos != nullptr) {
    	delete goToPos;
    }
    goToPos = new GoToPos("", bb);
}

bt::Node::Status FollowPath::Update () {

	roboteam_msgs::World world = LastWorld::get();

	double xGoal = GetDouble("xGoal");
    double yGoal = GetDouble("yGoal");
    double wGoal = GetDouble("wGoal");
    int robotID = blackboard->GetInt("ROBOT_ID");

	while (world.us.size() == 0) {
		return Status::Running;
	}

	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
	roboteam_utils::Vector2 goalPos = roboteam_utils::Vector2(xGoal, yGoal);

    switch (state) {
	case COMPUTE:
		points = ComputePath(robotPos, goalPos);
		state = GOTO;
        break;
	case GOTO:
		CallGoToPos(points[0], wGoal, robotID);
		state = CHECK;
        break;
	case CHECK:
		if (goToPos == nullptr) {
			ROS_INFO("Oh no! Null Pointer :(");
			return Status::Invalid;
		}

		if (goToPos->Update() == bt::Node::Status::Success) {
			if (points.size() > 1) {
				points.erase(points.begin());
				// ROS_INFO("Moving towards the next position target...");

				ROS_INFO_STREAM("X: " << points[0].x << ", Y: " << points[0].y);
				state = GOTO;
				return Status::Running;
			} else {
				ROS_INFO("Reached the last position target! Shutting down...");
				return Status::Success;
			}
		} else {
			if (goToPos->Update() == bt::Node::Status::Running) {
				return Status::Running;
			}
			if (goToPos->Update() == bt::Node::Status::Failure) {
				return Status::Failure;
			}
		}
        break;
    default:
        ROS_ERROR("Incomplete case statement for FollowPath::state in FollowPath::update!");
        return Status::Invalid;
    }

	ROS_INFO("You shouldn't be here...");
	return Status::Invalid;
}

} //rtt
