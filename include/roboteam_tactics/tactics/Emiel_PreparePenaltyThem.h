#pragma once

#include "roboteam_tactics/Parts.h"
#include <boost/uuid/uuid.hpp>

namespace rtt {

	class Emiel_PreparePenaltyThem : public Emiel_Prepare {
	public:
		Emiel_PreparePenaltyThem(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();
		Status Update();

		ros::NodeHandle nh;

	private:
		std::vector<boost::uuids::uuid> tokens;
	};

}