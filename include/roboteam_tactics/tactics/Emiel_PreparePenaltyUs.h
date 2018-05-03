#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

	class Emiel_PreparePenaltyUs : public Tactic {
	public:
		Emiel_PreparePenaltyUs(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();
		Status Update();

		ros::NodeHandle nh;

	private:
		std::vector<boost::uuids::uuid> tokens;
	};

}