#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/LastWorld.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class AimSkill : public Skill {

private:
	roboteam_msgs::SteeringGoal prevgoal;
public:
	AimSkill(Aggregator& aggregator) : 
			Skill{aggregator} {
	}
	
	double cleanAngle(double angle){
		if (angle < M_PI){
			return fmod(angle-M_PI, (2*M_PI))+M_PI;
		}
		else if(angle > M_PI){
			return fmod(angle+M_PI, (2*M_PI))-M_PI;
		}
		else {
			return angle;
		}
	
	}
	
	Status Update (){
		double aimangle=181*M_PI/180;
		
		aimangle=cleanAngle(aimangle);	

		roboteam_msgs::World world = LastWorld::get();
		roboteam_msgs::WorldBall ball = world.ball;
		
		if (world.robots_yellow.size() == 0){
			return Status::Running;
		}
		roboteam_msgs::WorldRobot robot = world.robots_yellow[0];
	
		roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(ball.pos.x, ball.pos.y);
		roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robot.pos.x, robot.pos.y);
		roboteam_utils::Vector2 posDiff = ballPos-robotPos;		
		
		roboteam_msgs::SteeringGoal goal;
		
		
		double rotDiff=aimangle-robot.w;
		//if (rotDiff > M_PI){rotDiff=rotDiff-2*M_PI;}
		//if (rotDiff < M_PI){rotDiff=rotDiff+2*M_PI;}
		
		rotDiff=cleanAngle(rotDiff);
		if (posDiff.length() < 0.105) { // got ball
			if (rotDiff < 0.05 and rotDiff > -0.05){ // correct orientation
				ROS_INFO("Target position and angle reached, finished");
				return Status::Success;
			}
			else {
				double rotPconstant=1; // for more speed, upping this constant is not the answer because the robot will cut its rotation short, instead implement a target veloctiy besides a target position
        		double maxrotv=1;
				double requiredrotchange=rotDiff*rotPconstant;
				
				if(requiredrotchange>maxrotv){
					requiredrotchange=maxrotv;
				}
				else if(requiredrotchange<-maxrotv){
					requiredrotchange=-maxrotv;
				}
				roboteam_utils::Vector2 aimvector;
				aimvector.x=cos(robot.w+requiredrotchange);
				aimvector.y=sin(robot.w+requiredrotchange);
				
				roboteam_utils::Vector2 targetPos = ballPos-aimvector.scale(0.1);
				goal.dribbler=true;
				goal.x = targetPos.x;
				goal.y = targetPos.y;
				goal.orientation=aimvector.angle();
				
				if(goal.x != prevgoal.x or goal.y != prevgoal.y or goal.orientation != prevgoal.orientation){
					prevgoal=goal;
					aggregator.putMsg(0,goal);
				}
				return Status::Running;
			}
			
			
		} 
		else if (posDiff.length() < 0.3) { // reasonably close, drive towards it
			double targetAngle = posDiff.angle();

			roboteam_utils::Vector2 posDiffNorm = posDiff.normalize();
			roboteam_utils::Vector2 targetPos = ballPos - posDiffNorm.scale(0.10);
			goal.x = targetPos.x;
			goal.y = targetPos.y;
			goal.orientation = targetAngle;
			if(goal.x != prevgoal.x or goal.y != prevgoal.y or goal.orientation != prevgoal.orientation){
				ROS_INFO("drive to");
				prevgoal=goal;
				aggregator.putMsg(0,goal);
			}
			return Status::Running;
		}	
		else { // to far from ball, use other options
			ROS_INFO("cannot rotate around ball, not near ball");
			return Status::Failure;
		}
		
	}
};

} // rtt

void msgCallBack(const roboteam_msgs::World world) {
	rtt::LastWorld::set(world);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "AimSkill");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBack);

	rtt::Aggregator aggregator;
	rtt::AimSkill aimSkill(aggregator);
	while (ros::ok()) {
		ros::spinOnce();
		if(aimSkill.Update() != bt::Node::Status::Running){break;}
	}

	return 0;
}
