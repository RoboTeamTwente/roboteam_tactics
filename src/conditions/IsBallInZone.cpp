#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/conditions/TeamHasBall.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsBallInZone.h"

namespace rtt {

IsBallInZone::IsBallInZone(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status IsBallInZone::Update() {
	roboteam_msgs::World world = LastWorld::get();
	auto field = LastWorld::get_field();
	Vector2 ballPos(world.ball.pos.x, world.ball.pos.y);
	std::string our_side = get_our_side();
	
	int zone=int(private_bb->GetDouble("zone"));
	
	// zone1 is as seen from the goal: right rear
	double zone1x1=4.5;
	double zone1x2=1.5;
	double zone1y1=3.0;
	double zone1y2=0.0;
	
	// zone2 is as seen from the goal: left rear
	double zone2x1=4.5;
	double zone2x2=1.5;
	double zone2y1=0.0;
	double zone2y2=-3.0;

	// zone 3: all field except area close to opponents goal
	double zone3x1=3.5;
	double zone3x2=-4.5;
	double zone3y1=-3.0;
	double zone3y2=-3.0;
	
	if(zone == 1){
		if(our_side == "right"){
			if(ballPos.x < zone1x1 && ballPos.x > zone1x2 && ballPos.y < zone1y1 && ballPos.y > zone1y2){
				return Status::Success;
			}
		}
		else{
			if(ballPos.x > -zone1x1 && ballPos.x < -zone1x2 && ballPos.y > -zone1y1 && ballPos.y < -zone1y2){
				return Status::Success;
			}
		
		}
	} else if(zone == 2){
		if(our_side == "right"){
			if(ballPos.x < zone2x1 && ballPos.x > zone2x2 && ballPos.y < zone2y1 && ballPos.y > zone2y2){
				return Status::Success;
			}
		}
		else{
			if(ballPos.x > -zone2x1 && ballPos.x < -zone2x2 && ballPos.y > -zone2y1 && ballPos.y < -zone2y2){
				return Status::Success;
			}
		
		}
	
	
	}
	else if(zone == 3){
		if(our_side == "right"){
			if(ballPos.x < zone3x1 && ballPos.x > zone3x2 && ballPos.y < zone3y1 && ballPos.y > zone3y2){
				return Status::Success;
			}
		}
		else{
			if(ballPos.x > -zone3x1 && ballPos.x < -zone3x2 && ballPos.y > -zone3y1 && ballPos.y < -zone3y2){
				return Status::Success;
			}
		
		}
	
	
	}
	return Status::Failure;
}

}
