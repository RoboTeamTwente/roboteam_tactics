#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/Math.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/DefendGoalarea.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG DefendGoalarea

namespace rtt {

RTT_REGISTER_SKILL(DefendGoalarea);

DefendGoalarea::DefendGoalarea(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard), 
        rotate_bb(std::make_shared<bt::Blackboard>()), 
        goto_bb(std::make_shared<bt::Blackboard>()), 
        rotateAroundPoint("", rotate_bb), 
        goToPos("", goto_bb) { 


        }



bt::Node::Status DefendGoalarea::Update() {
	ros::param::get("our_side", our_side);


	double spacingfromline=0.05;

	int robotID = blackboard->GetInt("ROBOT_ID");
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_msgs::WorldBall ball = world.ball;
	
	// check for robot position to see which part of the goalarea to follow
	if(robot.pos.y > -0.25 and robot.pos.y < 0.25){ // on the straight
		ROS_INFO("middle circle");
		goto_bb->SetInt("ROBOT_ID", robotID);
		
		goto_bb->SetBool("dribbler",false);
		goto_bb->SetDouble("maxspeed",2.0);
		if (our_side == "right"){
			goto_bb->SetDouble("xGoal", 3.5-spacingfromline);
			goto_bb->SetDouble("angleGoal", 3.14);
		} else {
			goto_bb->SetDouble("xGoal", -3.5+spacingfromline);
			goto_bb->SetDouble("angleGoal", 0.0);
		}
		goto_bb->SetDouble("yGoal", ball.pos.y);
		goto_bb->SetBool("endPoint",true);
		
		goToPos.Update();
	}
	else { // on an arc
		if (our_side == "right"){ // right field side
			rotate_bb->SetDouble("centerx", 4.5);
		} else {
			rotate_bb->SetDouble("centerx", -4.5);
		}

		if(robot.pos.y > 0.25){ // top half circle
			rotate_bb->SetDouble("centery", 0.25);
			ROS_INFO("top circle");
		} else { // bottom half circle
			rotate_bb->SetDouble("centery", -0.25);
			ROS_INFO("bottom circle");
		}

		rotate_bb->SetInt("ROBOT_ID", robotID);
		rotate_bb->SetString("center", "point");
		
		rotate_bb->SetDouble("radius", 1.00+spacingfromline);
	    rotate_bb->SetDouble("faceTowardsPosx", ball.pos.x);
	    rotate_bb->SetDouble("faceTowardsPosy", ball.pos.y);
	    rotate_bb->SetDouble("w", 2.0);
	    rotate_bb->SetBool("faceoutward", true);
	    rotate_bb->SetBool("drivetocircle",true);
	    rotateAroundPoint.Update();

	}

	
    return Status::Running;
}

} // rtt
