#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

	class Emiel_PrepareKickoffThem : public Tactic {
	public:
		Emiel_PrepareKickoffThem(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();
		Status Update();

		ros::NodeHandle nh;

	private:
		std::vector<boost::uuids::uuid> tokens;
	};

}