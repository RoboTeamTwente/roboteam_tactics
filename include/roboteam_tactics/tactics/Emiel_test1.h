#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"


namespace rtt {

/**
 * # Using the YAML multiline literal here
 * Descr: |
 *     Tests the creation of a new play.
 *
 * Params:
 *     - aParam
 *         Type: Int
 *         Descr: Just some parameter, to test if this works
 */

    class Emiel_test1 : public Tactic {
    public:
        Emiel_test1(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        void Initialize();
        Status Update();
        void Terminate(Status s);

        ros::NodeHandle nh;
    };
}