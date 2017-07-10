#pragma once

#include "roboteam_tactics/Parts.h"
#include "unique_id/unique_id.h"

namespace rtt {

class PenaltyThemPlay : public Tactic {
public:
	PenaltyThemPlay(std::string name, bt::Blackboard::Ptr bb);
	void Initialize() override;
	Status Update() override;
private:
	int shooterId;
	bool shootoutPenalty;
	bool valid;
};

}
