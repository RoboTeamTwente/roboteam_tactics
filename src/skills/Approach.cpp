#include "roboteam_tactics/skills/Approach.h"
#include <signal.h>

namespace rtt {

Approach::Approach(std::string name, bt::Blackboard::Ptr bb) : Skill(name, blackboard) {}

void trap(int) {
    ROS_INFO("segfault!");
}

bt::Node::Status Approach::Update() {
    //signal(SIGSEGV, trap);
    if (!gtp) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        ROS_INFO("merging: %p %p", blackboard.get(), bb.get());
        //merge_blackboards(bb, blackboard);
        ROS_INFO("building gtp");
        gtp = std::make_unique<GoToPos>(name, bb);
        ROS_INFO("gtp built: %p", gtp.get());
    }
    signal(SIGSEGV, 0);
    auto gtpStatus = gtp->Update();
    if (gtpStatus != bt::Node::Status::Running) {
        return gtpStatus;
    }
    Vector2 goal(GetDouble("xGoal"), GetDouble("yGoal"));
    double dist = fabs(goal.dist(Vector2(getWorldBot(GetInt("ROBOT_ID"))->pos)));
    bool done = dist < HasDouble("approachDistance") ? GetDouble("approachDistance") : DEFAULT_DISTANCE;
    return done ? bt::Node::Status::Success : bt::Node::Status::Running;
}
    
}