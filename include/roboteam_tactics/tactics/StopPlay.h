#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class StopPlay : public Tactic {
public:
	StopPlay(std::string name, bt::Blackboard::Ptr bb = nullptr);
	Status Update() override;
	void Initialize() override;
private:
	bool canRun;
};

}
