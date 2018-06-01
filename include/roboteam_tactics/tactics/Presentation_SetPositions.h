#pragma once

#include "roboteam_tactics/tactics/Emiel_Prepare.h"
#include "roboteam_tactics/Parts.h"
#include <boost/uuid/uuid.hpp>

namespace rtt {

	class Presentation_SetPositions : public Emiel_Prepare {
	public:
		Presentation_SetPositions(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();
		Status Update();

		ros::NodeHandle nh;

	private:
		std::vector<boost::uuids::uuid> tokens;
	};

}
