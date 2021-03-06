#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_utils/Draw.h"

namespace rtt {

class Anouk_MultipleDefendersPlay : public Tactic {

    public:
    Anouk_MultipleDefendersPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    static boost::optional<int> getClosestDefender(std::vector<int> robots,	roboteam_msgs::World& world, Vector2 dangerPos, double angleOffset);
    static std::vector<int> assignRobotsToPositions(std::vector<int> robots, std::vector<Vector2> points, roboteam_msgs::World& world);

	boost::uuids::uuid init_robotDefender(int robotID, int opponentID);
	boost::uuids::uuid init_ballDefender(int robotID, float angleOffset, Vector2 GoToPos_A_pos);

    void Initialize();
    Status Update();

    private:
    bool reInitializeWhenNeeded(bool forceInitialize);
    // void reInitialize(int newNumBallDefenders, int newNumRobotDefenders, std::vector<roboteam_msgs::WorldRobot> dangerousOpps);
    // void reInitialize();

    ros::NodeHandle n;

    std::vector<boost::uuids::uuid> tokens;

    time_point start;

    std::vector<roboteam_msgs::WorldRobot> dangerOrder;
    std::vector<roboteam_msgs::WorldRobot> prevDangerOrder;

	std::vector<int> prevDangerousOpps;

    std::vector<int> activeRobots;

    int numBallDefenders;
    int numRobotDefenders;
	int numExtraDefenders;
    // double prevDistBallToGoal;
    Vector2 prevBallPos;
    double distBallToGoalThreshold;

	bool weAreAttacking;
    bool weWereAttacking;
    int weAreAttackingCounter;
    int prevNumRobots;

    Draw drawer;
};

}
