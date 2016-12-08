#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/BackUp.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/SkillFactory.h"
#include <cmath>
#include <algorithm>

#define BACK_UP_DIST .3

namespace rtt {
    
RTT_REGISTER_SKILL(BackUp);

BackUp::BackUp(std::string name, bt::Blackboard::Ptr bb)
        : Skill(name, bb) {
    assert_valid<BackUp>(name);
    auto bot = *(lookup_bot(blackboard->GetInt("ROBOT_ID"), true));
    original_pos = roboteam_utils::Position(bot.pos.x, bot.pos.y, bot.angle);
    double theta = fmodl(original_pos.getRot(), 2*M_PI);
    double dx = cosl(theta) * BACK_UP_DIST;
    double dy = sinl(theta) * BACK_UP_DIST;
    target_pos = roboteam_utils::Position(original_pos.x - dx, original_pos.y - dy, original_pos.rot);
}

inline double limit(double x) {
    return x > .2 ? .2 : x < -.2 ? -.2 : x;
}
        
bt::Node::Status BackUp::Update() {
    auto bot = *(lookup_bot(blackboard->GetInt("ROBOT_ID"), true));
    roboteam_utils::Position current_pos = roboteam_utils::Position(bot.pos.x, bot.pos.y, bot.angle);
    
    roboteam_utils::Vector2 vec = target_pos.location() - current_pos.location();
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    
    if (vec.length() < .101) {
        pub.publish(stop_command(blackboard->GetInt("ROBOT_ID")));
        return bt::Node::Status::Success;
    }
    
    if (!dribble) {
       dribble = build_skill<Dribble>("Dribble", "", "ROBOT_ID=%d goalx=%f goaly=%f", 
            blackboard->GetInt("ROBOT_ID"), target_pos.x, target_pos.y);
    }
    
    roboteam_msgs::RobotCommand cmd;
    cmd.id = bot.id;
    cmd.active = true;
    cmd.dribbler = true;
    cmd.x_vel = limit(vec.x);
    cmd.y_vel = limit(vec.y);
    ROS_INFO("from: (%f,%f) to: (%f,%f) vel: (%f,%f)", current_pos.x, current_pos.y, target_pos.x, target_pos.y, cmd.x_vel, cmd.y_vel);
    cmd.kicker = false;
    cmd.chipper = false;
    
    pub.publish(cmd);
    return bt::Node::Status::Running;
}
    
}
