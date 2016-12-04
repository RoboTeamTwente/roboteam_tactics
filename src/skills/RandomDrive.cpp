#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>
#include <math.h>

#include "ros/ros.h"

#include "roboteam_tactics/skills/RandomDrive.h"
#include "roboteam_msgs/DebugPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"


namespace rtt {

RTT_REGISTER_SKILL(RandomDrive);

RandomDrive::RandomDrive(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , avoidRobots("", private_bb) {
	debug_pub = n.advertise<roboteam_msgs::DebugPoint>("view_debug_points", 1000);

    // Generate a random id for this RandomDriver.
    debug_id = get_rand_int(100000);

    pick_new_goal = true;
}

bt::Node::Status RandomDrive::Update() {
    roboteam_msgs::World world = LastWorld::get();

    int robotID = blackboard->GetInt("ROBOT_ID");

    if (world.us.size() == 0) {
        // Apparently there are no robots yet.
        return Status::Running;
    }

    if (pick_new_goal) {
        float half_field_width = LastWorld::get_field().field_width/2;
        float half_field_length = LastWorld::get_field().field_length/2;

        goal.x = get_rand_real(-half_field_length, half_field_length);
        goal.y = get_rand_real(-half_field_width, half_field_width);

        goal_angle = get_rand_real(-M_PI, M_PI);

        roboteam_msgs::DebugPoint point = roboteam_msgs::DebugPoint();
        point.name = "random drive " + std::to_string(debug_id);
        point.pos.x = goal.x;
        point.pos.y = goal.y;
        point.color.r = 255;
        point.color.g = 0;
        point.color.b = 255;
        debug_pub.publish(point);

        pick_new_goal = false;
    }

    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetDouble("xGoal", goal.x);
    private_bb->SetDouble("yGoal", goal.y);
    private_bb->SetDouble("angleGoal", goal_angle);

    if (avoidRobots.Update() == Status::Success) {
        pick_new_goal = true;
    }

    return Status::Running;
}

} // rtt
