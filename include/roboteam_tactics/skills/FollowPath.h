#pragma once

#include <vector>
#include "roboteam_utils/Vector2.h"

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class FollowPath
 * \brief See YAML
 */
/**
 * # Using the YAML multiline literal here
 * Descr: Goes to a position
 * Params:
 *   - ROBOT_ID:
 *       Descr:     Id of the robot
 *       Type:      Int
 *   - wGoal:
 *       Descr:     The angle of the arrival position
 *       Type:      Double
 *   - xGoal:
 *       Descr:     The target x coordinate
 *       Type:      Double
 *   - yGoal:
 *       Descr:     The target y coordinate
 *       Type:      Double
 */
 
// Forward declare skill
class GoToPos;

class FollowPath : public Skill {
public:
    FollowPath(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    std::vector<roboteam_msgs::Vector2f> ComputePath(Vector2 robotPos, Vector2 goalPos);
    void CallGoToPos(roboteam_msgs::Vector2f point, double wGoal, int robotID);
	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["xgoal"] = BBArgumentType::Double;
        params["ygoal"] = BBArgumentType::Double;
        params["wgoal"] = BBArgumentType::Double;
        return params;
    }
    
    std::string node_name() { return "FollowPath"; }
private:

    enum FPState { COMPUTE, GOTO, CHECK };

	ros::NodeHandle n;
	ros::ServiceClient client;
	std::vector<roboteam_msgs::Vector2f> points;

	GoToPos* goToPos;
	int robotID;
	FPState state; // 1: compute path, 2: go to a position along the path, 3: check if done, or move to next position
} ;

} // rtt
