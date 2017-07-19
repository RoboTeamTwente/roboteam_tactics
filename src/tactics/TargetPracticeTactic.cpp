#include "roboteam_tactics/tactics/TargetPracticeTactic.h"

#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {
    
RTT_REGISTER_TACTIC(TargetPracticeTactic);    
    
TargetPracticeTactic::TargetPracticeTactic(std::string name, bt::Blackboard::Ptr bb)
        : Tactic(name, bb) {}
        
void TargetPracticeTactic::Initialize() {
    tokens.clear();
    
    if (!(valid = getAvailableRobots().size() >= 2)) {
        return;
    }
    
    std::vector<int> robots = getAvailableRobots();

    int keeperID = robots.at(0);
    int shooterID = robots.at(1);
    
    RobotDealer::claim_robots({keeperID, shooterID});
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();
    {
        bt::Blackboard bb;
        ScopedBB(bb, "Kick_A")
            .setDouble("kickVel", 6.0)
            .setBool("wait_for_signal",false);
        ScopedBB(bb, "GetBall_A")
            .setBool("intercept", false)
            .setBool("isKeeper", true)
            .setString("AimAt", "robot")
            .setInt("AimAtRobot", shooterID)
            .setBool("passOn", true);
        bb.SetDouble("GetBall_B_getBallAtX", -4.2);
        bb.SetDouble("GetBall_B_getBallAtY", 0);
        bb.SetDouble("GetBall_B_getBallAtTime", 5.0);
        bb.SetBool("GetBall_B_intercept", true);
        bb.SetDouble("GetBall_B_acceptableDeviation", 0.45);
        bb.SetBool("GetBall_B_isKeeper", true);
        bb.SetBool("GetBall_B_passOn", true);
        bb.SetInt("ROBOT_ID", keeperID);
        bb.SetInt("KEEPER_ID", keeperID);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = keeperID;
        wd.tree = "rtt_bob/BasicKeeperTree";
        wd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
    {
        bt::Blackboard bb;
        ScopedBB(bb, "GetBall_A")
            .setString("AimAt", "ourgoal")
            .setBool("passOn", true)
            .setBool("intercept", false)
            .setBool("isKeeper", false)
            .setBool("ignoreFieldBounds", true);
        bb.SetInt("ROBOT_ID", shooterID);
        bb.SetInt("KEEPER_ID", keeperID);
        roboteam_msgs::RoleDirective wd;
        wd.robot_id = shooterID;
        wd.tree = "rtt_dennis/TargetPracticeRole";
        wd.blackboard = bb.toMsg();
        boost::uuids::uuid token = unique_id::fromRandom();
        tokens.push_back(token);
        wd.token = unique_id::toMsg(token);
        pub.publish(wd);
    }
}
   
bt::Node::Status TargetPracticeTactic::Update() {
    return valid ? bt::Node::Status::Running : bt::Node::Status::Invalid;
}
 
}
