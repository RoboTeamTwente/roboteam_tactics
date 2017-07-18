#pragma once

#include <chrono>
#include <ros/ros.h>

#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_utils/Draw.h"
#include "roboteam_tactics/skills/GoToPos.h"

namespace rtt {

/**
 * \class FindPath
 * \brief See YAML
 */
/*
 * Descr: Finds a path and drives it.
 * Params:
 *   - xGOal:
 *       Type: Double
 *       Descr: x coordinate of the goal
 *   - yGOal:
 *       Type: Double
 *       Descr: y coordinate of the goal
 */
class FindPath : public Skill {
public:
	FindPath(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ms"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "FindPath"; }

    using Clock = std::chrono::steady_clock;

private:
    void findPath();
    bt::Node::Status vanillaGoToPos();   
    bt::Node::Status followCurrentPath();
    bt::Node::Status runGoToPos(Vector2 pos);

    Clock::time_point lastPathSearch;
    unsigned int pointIndex;
    std::vector<Vector2> currentPath;
    time_point start;

    Draw draw;
    std::shared_ptr<bt::Blackboard> gtpBb;
    GoToPos gtp;

    bool wroteToFile = false;
};


} // rtt
