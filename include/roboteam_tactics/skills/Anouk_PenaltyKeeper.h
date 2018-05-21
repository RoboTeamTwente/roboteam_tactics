#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

#include "roboteam_tactics/skills/GoToPos.h"

namespace rtt {

class Anouk_PenaltyKeeper : public Skill {
public:
    Anouk_PenaltyKeeper(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();
    Status Update();

	void publishStopCommand(int robotID);
	Vector2 calculateKeeperPosition();

    std::string node_name() override { return "Anouk_PenaltyKeeper"; }

private:
    Vector2 keeperPosition;
	GoToPos goToPos;

} ;

} // rtt

