#include "roboteam_tactics/tactics/WanderTactic.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"

#define RTT_CURRENT_DEBUG_TAG WanderTactic

namespace rtt {
    
RTT_REGISTER_TACTIC(WanderTactic);

WanderTactic::WanderTactic(std::string name, bt::Blackboard::Ptr bb) : Tactic(name, bb) {}

bool fill(std::vector<int>& target, std::vector<int>& source, int max) {
    int avail = source.size();
    if (avail <= max) {
        target.insert(target.begin(), source.begin(), source.end());
        return false;
    }
    for (int i = 1; i <= max; i++) {
        target.push_back(source.at(source.size() - 1));
        source.pop_back();
    }
    return true;
}

void WanderTactic::Initialize() {
    tokens.clear();
    
    std::vector<int> available = getAvailableRobots();
    
    if (available.size() == 0) {
        valid = false;
        return;
    }
    
    std::vector<int> nearBall, nearGoal, firstQuad, secondQuad;
    int wantNearBall = GetInt("nearBall", DEFAULT_NEAR_BALL);
    int wantNearGoal = GetInt("nearGoal", DEFAULT_NEAR_GOAL);
    int wantInFirstQuad = GetInt("mostImportantQuad", DEFAULT_MOST_IMPORTANT_QUADRANT);
    int wantInSecondQuad = GetInt("secondaryQuad", DEFAULT_SECONDARY_QUARDRANT);
    
    bool cont = fill(nearBall, available, wantNearBall);
    if (cont) cont = fill(nearGoal, available, wantNearGoal);
    if (cont) cont = fill(firstQuad, available, wantInFirstQuad);
    if (cont) cont = fill(secondQuad, available, wantInSecondQuad);
    
    claim_robots(nearBall);
    claim_robots(nearGoal);
    claim_robots(firstQuad);
    claim_robots(secondQuad);
    
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    
    int keeper = -1;
    for (int id : nearGoal) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        bb->SetInt("ROBOT_ID", id);
        bb->SetInt("KEEPER_ID", keeper >= 0 ? keeper : (keeper = id));
        bb->SetString("Wander_A_type", "NEAR_OUR_GOAL");
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = id;
        wd.tree = "rtt_dennis/WanderRole";
        wd.blackboard = bb->toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
    for (int id : nearBall) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        bb->SetInt("ROBOT_ID", id);
        bb->SetInt("KEEPER_ID", keeper);
        bb->SetString("Wander_A_type", "NEAR_BALL");
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = id;
        wd.tree = "rtt_dennis/WanderRole";
        wd.blackboard = bb->toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
    int quad1 = pickMostImportantQuad();
    int quad2 = quad1 % 2 ? quad1 - 1 : quad1 + 1;
    
    for (int id : firstQuad) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        bb->SetInt("ROBOT_ID", id);
        bb->SetInt("KEEPER_ID", keeper);
        bb->SetString("Wander_A_type", "QUADRANT");
        bb->SetInt("Wander_A_quadrant", quad1);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = id;
        wd.tree = "rtt_dennis/WanderRole";
        wd.blackboard = bb->toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    
    for (int id : secondQuad) {
        bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
        bb->SetInt("ROBOT_ID", id);
        bb->SetInt("KEEPER_ID", keeper);
        bb->SetString("Wander_A_type", "QUADRANT");
        bb->SetInt("Wander_A_quadrant", quad2);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = id;
        wd.tree = "rtt_dennis/WanderRole";
        wd.blackboard = bb->toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
}

int WanderTactic::pickMostImportantQuad() const {
    Vector2 ballPos(LastWorld::get().ball.pos);
    if ((ballPos.x <= 0 && weAreLeft()) || (ballPos.x > 0 && weAreLeft())) {
        return ballPos.y >= 0 ? 0 : 2;
    } else {
        return ballPos.y >= 0 ? 1 : 3;
    }
}

bt::Node::Status WanderTactic::Update() {
    if (!valid) return bt::Node::Status::Invalid;
    return bt::Node::Status::Running;
}
    
}
