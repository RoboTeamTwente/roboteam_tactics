#pragma once

#include "roboteam_tactics/Leaf.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class CanSeePoint
 * \brief See YAML
 */
 /*
  * Descr: Checks whether a robot can see a certain point
  * Params:
  *   - ROBOT_ID:
  *       Type: Int
  *       Descr: The robot to check for
  *   - x_coor: 
  *       Type: Double
  *       Descr: The x-coordinate of the target location
  *   - y_coor:
  *       Type: Double
  *       Descr: The y-coordinate of the target location
  *   - check_move:
  *       Type: Bool
  *      Descr: Whether it should check for possibility of movement or just line of sight.
  */
class CanSeePoint : public Condition {
  
    public:
    static VerificationMap required_params() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        params["x_coor"] = BBArgumentType::Double;
        params["y_coor"] = BBArgumentType::Double;
        params["check_move"] = BBArgumentType::Bool;
        return params;
    }
    
    CanSeePoint(std::string name, bt::Blackboard::Ptr blackboard);
    Status Update() override;
    std::string node_name() override { return "CanSeePoint"; }
    
    protected:
    double threshold_dist;
    
};

// TODO: If anyone is using this class at some point, please factor it into
// it's own source + header file. At this moment using skills like this is
// not possible (i.e. will cause build errors probably)
class CanSeeGoal : public CanSeePoint {
    public:
    CanSeeGoal(std::string name, bt::Blackboard::Ptr blackboard) : CanSeePoint(name, blackboard) {
        assert_valid<CanSeeGoal>(name);
        Vector2 a, b; //TODO: goal positions
        std::string name_a = name + "_can_see_goal_a";
        std::string name_b = name + "_can_see_goal_b";
        blackboard->SetInt(name_a + "_me", GetInt("me"));
        blackboard->SetDouble(name_a + "_x_coor", a.x);
        blackboard->SetDouble(name_a + "_y_coor", a.y);
        blackboard->SetBool(name_a + "check_move", true);
        blackboard->SetInt(name_b + "_me", GetInt("me"));
        blackboard->SetDouble(name_b + "_x_coor", b.x);
        blackboard->SetDouble(name_b + "_y_coor", b.y);
        blackboard->SetBool(name_b + "check_move", true);
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
        /*if (goal_a->Update() == Status::Invalid || goal_b->Update() == Status::Invalid) {
            return Status::Invalid;
        }*/
        return Status::Failure;
    }
        
        
    std::string node_name() { return "CanSeeGoal"; }
    static VerificationMap required_params() { return CanSeePoint::required_params(); }
    
    private:
    CanSeePoint* goal_a, *goal_b;
};

}
