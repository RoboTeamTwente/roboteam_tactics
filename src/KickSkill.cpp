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

class KickSkill : public Skill {

private:
	roboteam_msgs::SteeringGoal prevgoal;
public:
	KickSkill(Aggregator& aggregator) : 
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
		double kickfoce=1;
		
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

		double rotDiff=posDiff.angle()-robot.w;
		rotDiff=cleanAngle(rotDiff);
		if (posDiff.length() < 0.105) { // ball is close
			if(rotDiff < 0.1 and rotDiff > -0.1){ // ball in front
				goal.dribbler=false;
				goal.kick_forced=true;
				goal.x = robotPos.x;
				goal.y = robotPos.y;
				goal.orientation=robot.w;
				
				//if(goal.x != prevgoal.x or goal.y != prevgoal.y or goal.orientation != prevgoal.orientation){
				//	prevgoal=goal;
					aggregator.putMsg(0,goal);
				//}
				return Status::Success;
			}
			else {
				ROS_INFO("ball not in front of dribbler");
				return Status::Failure;
			}
			
		}
		else {
			ROS_INFO("ball not close to robot");
			return Status::Failure;
		} 
		
	}
};

} // rtt

void msgCallBack(const roboteam_msgs::World world) {
	rtt::LastWorld::set(world);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "KickSkill");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("world_state", 1000, msgCallBack);

	rtt::Aggregator aggregator;
	rtt::KickSkill kickSkill(aggregator);
	while (ros::ok()) {
		ros::spinOnce();
		if(kickSkill.Update() != bt::Node::Status::Running){break;}
	}

	return 0;
}
