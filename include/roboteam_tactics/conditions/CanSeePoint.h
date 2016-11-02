#pragma once

#include "roboteam_tactics/Leaf.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class CanSeePoint : public Condition {
  
    public:
    static VerificationMap required_params() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        params["x_coor"] = BBArgumentType::Double;
        params["y_coor"] = BBArgumentType::Double;
        return params;
    }
    
    CanSeePoint(std::string name, bt::Blackboard::Ptr blackboard);
    Status Update() override;
    
};

class CanSeeGoal : public CanSeePoint {
    public:
    CanSeeGoal(std::string name, bt::Blackboard::Ptr blackboard) : CanSeePoint(name, blackboard) {
        assert_valid<CanSeeGoal>(name);
        roboteam_utils::Vector2 a, b; //TODO: goal positions
        std::string name_a = name + "_can_see_goal_a";
        std::string name_b = name + "_can_see_goal_b";
        blackboard->SetInt(name_a + "_me", GetInt("me"));
        blackboard->SetDouble(name_a + "_x_coor", a.x);
        blackboard->SetDouble(name_a + "_y_coor", a.y);
        blackboard->SetInt(name_b + "_me", GetInt("me"));
        blackboard->SetDouble(name_b + "_x_coor", b.x);
        blackboard->SetDouble(name_b + "_y_coor", b.y);
        goal_a = new CanSeePoint(name_a, blackboard);
        goal_b = new CanSeePoint(name_b, blackboard);
    }
    
    ~CanSeeGoal() {
        delete goal_a;
        delete goal_b;
    }
        
    Status Update() override {
        if (goal_a->Update() == Status::Success || goal_b->Update() == Status::Success) {
            return Status::Success;
        }
        if (goal_a->Update() == Status::Invalid || goal_b->Update() == Status::Invalid) {
            return Status::Invalid;
        }
        return Status::Failure;
    }
        
    static VerificationMap required_params() { return CanSeePoint::required_params(); }
    
    private:
    CanSeePoint* goal_a, *goal_b;
};

}
