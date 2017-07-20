#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/WeHaveBall.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

/**
 * # Using the YAML multiline literal here
 * Descr: |
 *     Does kickoff defense.
 *
 * Params:
 *     - allowSuccess
 *         Type: Bool
 *         Descr: If true, returns success when the ball moves. Otherwise the play will run forever.
 */
class Jim_KickOffDefense : public Tactic {
    public:
    Jim_KickOffDefense(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Status Update();
    ros::NodeHandle n;

    private:
    std::vector<boost::uuids::uuid> tokens;
    time_point start;

    time_point lastUpdate;
    time_point finishTime;
    time_point lastTimeWeHadBall;

    WeHaveBall weHaveBall;
};

}
