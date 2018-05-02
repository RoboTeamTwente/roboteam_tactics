#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

	class Emiel_PrepareKickoffUs : public Tactic {
	public:
		Emiel_PrepareKickoffUs(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();
		Status Update();
		void Terminate(Status status);

		ros::NodeHandle nh;

	private:
		std::vector<boost::uuids::uuid> tokens;
	};

}