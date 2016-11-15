#include <string>

#include "ros/ros.h"

#include "roboteam_tactics/skills/RandomDrive.h"
#include "roboteam_msgs/DebugPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"


namespace rtt {

RandomDrive::RandomDrive(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        , avoidRobots(n, "", private_bb) {
	debug_pub = n.advertise<roboteam_msgs::DebugPoint>("view_debug_points", 1000);

    // Generate a random id for this RandomDriver.
    debug_id = get_rand_int(100000);

    float half_field_width = LastWorld::get_field().field_width/2;
    float half_field_height = LastWorld::get_field().field_length/2;

    goal_x = get_rand_real(-half_field_width, half_field_width);
    goal_y = get_rand_real(-half_field_height, half_field_height);
}

bt::Node::Status RandomDrive::Update() {
    roboteam_msgs::World world = LastWorld::get();

    int robotID = blackboard->GetInt("ROBOT_ID");

    if (world.us.size() == 0) {
        // Apparently there are no robots yet.
        return Status::Running;
    }

    roboteam_msgs::DebugPoint point = roboteam_msgs::DebugPoint();
    point.name = "random drive " + std::to_string(debug_id);
    point.pos.x = goal_x;
    point.pos.y = goal_y;
    point.color.r = 255;
    point.color.g = 0;
    point.color.b = 255;
    debug_pub.publish(point);
}

} // rtt
