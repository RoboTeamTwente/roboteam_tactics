#pragma once

#include "roboteam_tactics/Parts.h"
#include "unique_id/unique_id.h"

namespace rtt {

class PenaltyUsPlay : public Tactic {
public:
	PenaltyUsPlay(std::string name, bt::Blackboard::Ptr bb);
	void Initialize() override;
	Status Update() override;
private:
	int shooterId;
	bool shootoutPenalty;
	bool valid;
};

}
