#pragma once

#include <map>
#include <type_traits>
#include <vector>

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(World);
ROS_DECLARE_MESSAGE(RoleDirective);

} // roboteam_msgs

#include "roboteam_utils/constants.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/RobotDealer.h"
#include "roboteam_tactics/verifier.h"
#include "roboteam_tactics/Leaf.h"
#include "roboteam_tactics/utils/debug_print.h"

namespace rtt {

class Skill : public Leaf {
public:

    Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Skill();

    virtual Status Update();
} ;

class Condition : public Leaf {
	public:
	Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Condition();

    virtual Status Update();

    /**
     * @brief Returns a set of world states in which this Condition should succeed.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> success_states() const;

    /**
     * @brief Returns a set of world states in which this Condition should fail.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> fail_states() const;
} ;

class Tactic : public Leaf {
    public:
    Tactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Tactic();

    virtual void Initialize();
    virtual Status Update();
    virtual void Terminate(Status s);

    void claim_robot(int id);
    void claim_robots(std::vector<int> ids);
    std::vector<int> get_claimed_robots();
    bool is_claimed(int id);

    private:
    std::set<int> claimed_robots;
} ;

} // rtt
