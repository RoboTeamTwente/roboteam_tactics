#pragma once

#include <boost/optional.hpp>

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

/**
 * \class KeeperHasBall
 * \brief See YAML
 */
/*
 * Descr: Checks whether a robot has the ball
 * Params:
 *   - me:
 *       Type: Int
 *       Descr: The robot to check for
 *   - our_team:
 *       Type: Bool
 *       Descr: Whether or not the robot is on our team
 */
class KeeperHasBall : public Condition {


public:
    KeeperHasBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        params["our_team"] = BBArgumentType::Bool;
        return params;
    }
    
    std::vector<roboteam_msgs::World> success_states() const override;
    std::vector<roboteam_msgs::World> fail_states() const override;
    
    std::string node_name() override { return "KeeperHasBall"; }
private:
    int me;
    bool us;
    boost::optional<roboteam_msgs::WorldRobot> find_bot_pos(const roboteam_msgs::World&) const;
};
    
}
