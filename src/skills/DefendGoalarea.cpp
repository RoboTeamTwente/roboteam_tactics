#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/DefendGoalarea.h"
#include "roboteam_msgs/DebugPoint.h"

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
        	debug_pub = n.advertise<roboteam_msgs::DebugPoint>(TOPIC_DEBUG_POINTS, 1000);

        }


void DefendGoalarea::SetOffsets(){

	if(HasInt("numberOfRobots")){
		if(GetInt("numberOfRobots")==1){
			offsetlength=0.0;
			offsetangle=0.0;
		}
		else {
			offsetlength=robotradius;
			offsetangle=asin(robotradius/radius);

			if(GetInt("numberOfRobots")==2){
				if(GetString("position")=="top"){
					offsetangle=-offsetangle;
				}
				else {
					offsetlength=-offsetlength;
				}

			}
			else if(GetInt("numberOfRobots")==3){
				if(GetString("position")=="bottom"){
					offsetlength=-2*offsetlength;
					offsetangle=2*offsetangle;
				} else if(GetString("position")=="top"){
					offsetlength=2*offsetlength;
					offsetangle=-2*offsetangle;
				} else if(GetString("position")=="middle"){
					offsetlength=0.0;
					offsetangle=0.0;
				}


			}
			else {

				ROS_INFO("only 3 robots supported");
			}
		}


	}

}

bt::Node::Status DefendGoalarea::Update() {
	ros::param::get("our_side", our_side);
	
	double spacingfromline=0.05;
	radius=1.00+spacingfromline;
	SetOffsets();

	ROS_INFO("offsets: length:%f, angle:%f",offsetlength,offsetangle);

	int robotID = blackboard->GetInt("ROBOT_ID");
	roboteam_msgs::World world = LastWorld::get();

	roboteam_msgs::WorldRobot robot = world.us.at(robotID);
	roboteam_msgs::WorldBall ball = world.ball;
	
	bool forceonarc=false;
	// check for robot position to see which part of the goalarea to follow
	if (ball.pos.y < 0.25 && ball.pos.y > -0.25 && 
		(ball.pos.y+offsetlength > 0.25 || ball.pos.y+offsetlength < -0.25)){
		double straightpart=abs(ball.pos.y-0.25);
		double anglepart=offsetlength-straightpart;
		offsetangle=straightpart/offsetlength*offsetangle;

		forceonarc=true;

	}


	if(robot.pos.y > -0.25 && robot.pos.y < 0.25 && !forceonarc){ // on the straight
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
		goto_bb->SetDouble("yGoal", ball.pos.y+offsetlength);
		goto_bb->SetBool("endPoint",true);
		goto_bb->SetBool("avoidRobots", false);
		
		goToPos.Update();
	} else { // on an arc
		Vector2 center;
		if (our_side == "right"){ // right field side
			center.x=4.5;
		} else {
			center.x=-4.5;
		}

		if(robot.pos.y > 0){ // top half circle
			center.y=0.25;
			ROS_INFO("top circle");
		} else { // bottom half circle
			center.y=-0.25;
			ROS_INFO("bottom circle");
		}

		rotate_bb->SetDouble("centerx", center.x);
		rotate_bb->SetDouble("centery", center.y);
		rotate_bb->SetInt("ROBOT_ID", robotID);
		rotate_bb->SetString("center", "point");

		Vector2 ballpos = ball.pos;
		Vector2 centertoball=ballpos-center;
		centertoball = centertoball.rotate(offsetangle);
		Vector2 facetowardsPos=centertoball+center;

		roboteam_msgs::DebugPoint point = roboteam_msgs::DebugPoint();
        point.name = "face towards";
        point.pos.x = facetowardsPos.x;
        point.pos.y = facetowardsPos.y;
        point.color.r = 255;
        point.color.g = 0;
        point.color.b = 255;
        debug_pub.publish(point);

        ROS_INFO("facetowards x:%f y:%f",facetowardsPos.x,facetowardsPos.y);

		rotate_bb->SetDouble("radius", radius);
	    rotate_bb->SetDouble("faceTowardsPosx", facetowardsPos.x);
	    rotate_bb->SetDouble("faceTowardsPosy", facetowardsPos.y);
	    rotate_bb->SetDouble("w", 2.0);
	    rotate_bb->SetBool("faceoutward", true);
	    rotate_bb->SetBool("drivetocircle",true);
	    rotateAroundPoint.Update();
	}

	
    return Status::Running;
}

} // rtt
