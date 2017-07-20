#include <ros/ros.h>

#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"

#include "roboteam_tactics/conditions/AimingAtOpponent.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"

namespace rtt {
    
RTT_REGISTER_CONDITION(AimingAtOpponent);

AimingAtOpponent::AimingAtOpponent(std::string name, bt::Blackboard::Ptr blackboard) 
    : Condition(name, blackboard) { }

bt::Node::Status AimingAtOpponent::Update() {
    unsigned int id = GetInt("ROBOT_ID");
    double LINE_OF_SIGHT_LENGTH = 10;
    
    if (auto botOpt = getWorldBot(id)) {
        Vector2 pos = botOpt->pos;

        Vector2 botAngleVec(LINE_OF_SIGHT_LENGTH, 0);
        botAngleVec = botAngleVec.rotate(botOpt->angle);

        Vector2 endOfLineOfSight = pos + botAngleVec;

        private_bb->SetInt("ROBOT_ID", id);
        private_bb->SetDouble("x_coor", endOfLineOfSight.x);
        private_bb->SetDouble("y_coor", endOfLineOfSight.y);
        CanSeePoint csp("", private_bb);

        switch (csp.Tick()) {
            case Status::Success:
                return Status::Failure;
            case Status::Failure:
                return Status::Success;
            case Status::Running:
                return Status::Failure;
            case Status::Invalid:
                return Status::Failure;
        }   
    }

    return bt::Node::Status::Failure;
}
    
}
