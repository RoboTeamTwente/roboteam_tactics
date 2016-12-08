#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/tactics/BasicDefenseTactic.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#define RTT_CURRENT_DEBUG_TAG BasicDefenseTactic

namespace rtt {

RTT_REGISTER_TACTIC(BasicDefenseTactic);

BasicDefenseTactic::BasicDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard)
        : Tactic(name, blackboard) 
        {}

void BasicDefenseTactic::Initialize() {
    tokens.clear();

    RTT_DEBUG("Initializing\n");
    
    roboteam_msgs::World world = rtt::LastWorld::get();

    // Assign the remaining robots the secondary keeper role
    std::vector<int> robots = RobotDealer::get_available_robots();
    double yStep = LastWorld::get_field().field_width / (robots.size() + 1);
    double startY = LastWorld::get_field().field_width / 2 - yStep;

    // auto pub = rtt::get_roledirective_publisher();
    ros::NodeHandle n;
    auto pub = n.advertise<roboteam_msgs::RoleDirective>(TOPIC_ROLE_DIRECTIVE, 100);

    const Vector2 theirGoal = rtt::LastWorld::get_their_goal_center();

    for (const int ROBOT_ID : robots) {
        RTT_DEBUGLN("StartY: %f", startY);

        // Fill blackboard with relevant info
        bt::Blackboard bb;

        bb.SetInt("ROBOT_ID", ROBOT_ID);
        bb.SetDouble("xGoal", (LastWorld::get_our_goal_center().normalize() * 2).x);
        bb.SetDouble("yGoal", startY);
        bb.SetDouble("angleGoal", theirGoal.angle());

        RTT_DEBUGLN("xGoal: %f", bb.GetDouble("xGoal"));

        // Create message
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = ROBOT_ID;
        wd.tree = "AvoidRobots";
        wd.blackboard = bb.toMsg();

        // Send to rolenode
        pub.publish(wd);

        // Move one pos down
        startY -= yStep;

        RTT_DEBUG("Claiming: %i\n", ROBOT_ID);
    }

    claim_robots(robots);
}

bt::Node::Status BasicDefenseTactic::Update() {
    // Defense tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

