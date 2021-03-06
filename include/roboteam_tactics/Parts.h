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

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public Leaf {
public:

    Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Skill();

    virtual Status Update();
} ;

/**
 * \class Condition
 * \brief Base class for conditions.
 */
class Condition : public Leaf {
	public:
	Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Condition();

    virtual Status Update();

    /**
     * \brief Returns a set of world states in which this Condition should succeed.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> success_states() const;

    /**
     * \brief Returns a set of world states in which this Condition should fail.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> fail_states() const;
} ;

/**
 * \class Tactic
 * \brief Base class for tactics.
 */
class Tactic : public Leaf {
    public:
    Tactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Tactic();

    virtual void Initialize();
    virtual Status Update();
    virtual void Terminate(Status s);

    /**
     * \brief Registers a robot as claimed by this tactic. See RobotDealer.
     * \param id The id of the robot to claim.
     */
    virtual bool claim_robot(int id);
    
    /**
     * \brief Shortcut to claim multiple robots at once.
     * \param ids A vector of the ids to claim.
     */
    virtual bool claim_robots(std::vector<int> ids);
    
    /**
     * \brief Gets a vector of all robots claimed by this tactic.
     */
    virtual std::vector<int> get_claimed_robots();
    
    /**
     * \brief Checks wether a robot is claimed by this tactic.
     * \param id The id to check.
     */
    virtual bool is_claimed(int id);

    virtual bool release_robot(int id);
    virtual bool release_robots(std::vector<int> ids);

    private:
    std::set<int> claimed_robots;
} ;

/**
 * \class SingleBotTactic
 * \brief Utility class for "tactics" which only use a single robot.
 * It takes care of all the robot claiming stuff.
 */
class SingleBotTactic : public Tactic {
    public:
    SingleBotTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr)
            : Tactic(name, blackboard), bot(-1)
            {}
    virtual ~SingleBotTactic() {}
    
    bool claim_robot(int id) final override {
        if (bot == -1) {
            bot = id;
            return Tactic::claim_robot(id);
        } else {
            ROS_ERROR("SingleBotTactic %s tried to claim bot %d, but it has already claimed %d.",
                name.c_str(), id, bot);
            throw std::logic_error("SingleBotTactic multi-claim");
        }
    }
    bool claim_robots(std::vector<int> ids) final override {
        if (ids.size() == 0) return true;
        if (ids.size() == 1) {
            return claim_robot(ids.at(0));
        }
        ROS_ERROR("SingleBotTactic %s tried to claim multiple bots in one go.", name.c_str());
        throw std::logic_error("SingleBotTactic multi-claim");
    }
    
    std::vector<int> get_claimed_robots() final override {
        return bot == -1 ? std::vector<int>() : std::vector<int>({bot});
    }
    
    int get_claimed_robot() const {
        return bot;
    }
    
    bool is_claimed(int id) final override {
        return id == bot;
    }
        
    private:
    int bot;
};

// Make sure sources that include this are not troubled by the def
#undef RTT_CURRENT_DEBUG_TAG

} // rtt
