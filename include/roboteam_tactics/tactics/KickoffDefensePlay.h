#pragma once

#include "unique_id/unique_id.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

class KickoffDefensePlay : public Tactic {
public:
	KickoffDefensePlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
	Status Update() override;
private:
	std::vector<boost::uuids::uuid> tokens;
	time_point start;
};

}
