#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_msgs/RoleDirective.h"

namespace rtt {

class Jim_MultipleDefendersPlay : public Tactic {

    public:
    Jim_MultipleDefendersPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    static int getClosestDefender(std::vector<int> robots, roboteam_msgs::World& world, Vector2 dangerPos, double angleOffset);
    static std::vector<int> getClosestRobots(std::vector<int> robots, std::vector<Vector2> points, roboteam_msgs::World& world);

    void Initialize();
    Status Update();

    private:
    bool reInitializeWhenNeeded(bool justCheck);
    // void reInitialize(int newNumBallDefenders, int newNumRobotDefenders, std::vector<roboteam_msgs::WorldRobot> dangerousOpps);
    // void reInitialize();

    ros::NodeHandle n;

    std::vector<boost::uuids::uuid> tokens;

    time_point start;

    std::vector<roboteam_msgs::WorldRobot> dangerOrder;
    std::vector<roboteam_msgs::WorldRobot> prevDangerOrder;

    std::vector<int> activeRobots;

    int numBallDefenders;
    int numRobotDefenders;
    // double prevDistBallToGoal;
    Vector2 prevBallPos;
    double distBallToGoalThreshold;

    bool weWereAttacking;
    int weAreAttackingCounter;
    int prevNumRobots;
};

}