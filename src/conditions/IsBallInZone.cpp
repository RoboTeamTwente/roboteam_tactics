#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/IsBallInZone.h"

namespace rtt {

IsBallInZone::IsBallInZone(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

}

bt::Node::Status IsBallInZone::Update() {
	roboteam_msgs::World world = LastWorld::get();
	auto field = LastWorld::get_field();
	Vector2 ballPos(world.ball.pos.x, world.ball.pos.y);
	Vector2 point = ballPos;

	// THE CONDITION NEEDS DIFFERENT NAME, BECAUSE WE CAN PUT IN ROBOT POS AS WELL NOW

	// If robot pos should be checked instead of ball pos, get my position and use that as the point.
	if (HasBool("robot") && GetBool("robot")){
		int robotID = GetInt("ROBOT_ID");
		boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    	roboteam_msgs::WorldRobot me;
    	if (findBot) {
	        me = *findBot;
	    } else {
        	ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << robotID);
        	return Status::Invalid;
    	}
		point = me.pos;
	}

	//Initialize Zone
	double zone_x1=0.0;
	double zone_x2=0.0;
	double zone_y1=0.0;
	double zone_y2=0.0;

	//Check for zone number and set zone accordingly
	if (HasInt("zone")) {
		int zone=int(private_bb->GetInt("zone"));
		// zone1 is as seen from the goal: left rear
		if (zone==1) {
			zone_x1=-4.5;
			zone_x2=-1.5;
			zone_y1=3.0;
			zone_y2=0.0;
		}
		// zone2 is as seen from the goal: right rear
		else if (zone==2) {
			zone_x1=-4.5;
			zone_x2=-1.5;
			zone_y1=0.0;
			zone_y2=-3.0;
		} 
		// zone 3: all field except area close to opponents goal
		else if (zone==3) {
			zone_x1=-4.5;
			zone_x2=3.5;
			zone_y1=-3.0;
			zone_y2=3.0;
		}
		// zone 4: everywhere except the edges of the field (where the robot cant get the ball in case of demo field)
		else if (zone==4) {
			zone_x1=-field.field_length/2 + 0.2;
			zone_x2=field.field_length/2 - 0.2;
			zone_y1=-field.field_width/2 + 0.2;
			zone_y2=field.field_width/2 - 0.2;
		}
		// zone 5: zone 4 with larger margin
		else if (zone==5) {
			zone_x1=-field.field_length/2 + 0.35;
			zone_x2=field.field_length/2 - 0.35;
			zone_y1=-field.field_width/2 + 0.35;
			zone_y2=field.field_width/2 - 0.35;
		}

		else if (zone==6) {
			zone_x1=-4.8;
			zone_x2=-4.0;
			zone_y1=-1.5;
			zone_y2=1.5;
		}

	}
	
	// Set zone according to points given in blackboard
	if (HasDouble("x1")) {zone_x1 = GetDouble("x1");}
	if (HasDouble("x2")) {zone_x2 = GetDouble("x2");}
	if (HasDouble("y1")) {zone_y1 = GetDouble("y1");}
	if (HasDouble("y2")) {zone_y2 = GetDouble("y2");}
		
	// The condition
	if(point.x > zone_x1 && point.x < zone_x2 && point.y > zone_y1 && point.y < zone_y2){
		return Status::Success;
	}
	return Status::Failure;
}

}
