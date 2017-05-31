#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/tactics/FormationTacticBase.h"

namespace rtt {

class KeepFormation : public Skill {
public:
	KeepFormation(std::string name, bt::Blackboard::Ptr bb);
	Status Update() override;
};

}


