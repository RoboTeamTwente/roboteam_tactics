#include "roboteam_tactics/skills/Approach.h"

namespace rtt {

Approach::Approach(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb) {}

bt::Node::Status Approach::Update() {
    if (!gtp) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        merge_blackboards(bb, blackboard);
        gtp = std::make_unique<GoToPos>(name, bb);
    }
    auto gtpStatus = gtp->Update();
    if (gtpStatus != bt::Node::Status::Running) {
        gtp.reset(nullptr);
        return gtpStatus;
    }
    Vector2 goal(GetDouble("xGoal"), GetDouble("yGoal"));
    double dist = fabs(goal.dist(Vector2(getWorldBot(GetInt("ROBOT_ID"))->pos)));
    bool done = dist < HasDouble("approachDistance") ? GetDouble("approachDistance") : DEFAULT_DISTANCE;
    if (done) gtp.reset(nullptr);
    return done ? bt::Node::Status::Success : bt::Node::Status::Running;
}
    
}