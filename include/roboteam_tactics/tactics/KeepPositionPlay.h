#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class KeepPositionPlay : public Tactic {
public:
	KeepPositionPlay(std::string name, bt::Blackboard::Ptr bb);
	void Initialize() override;
	Status Update() override;
};

}

