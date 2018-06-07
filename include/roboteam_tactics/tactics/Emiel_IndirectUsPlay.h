#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastRef.h"
#include <boost/uuid/uuid.hpp>

namespace rtt {
	class Emiel_IndirectUsPlay : public Tactic {
	public:
		Emiel_IndirectUsPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();

		Status Update();

		ros::NodeHandle nh;

	private:
		std::vector<boost::uuids::uuid> tokens;

	};
} // namespace rtt
