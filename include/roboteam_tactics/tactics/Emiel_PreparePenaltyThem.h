#pragma once

#include "roboteam_tactics/Parts.h"
#include <boost/uuid/uuid.hpp>
#include "roboteam_utils/LastRef.h"

namespace rtt {

	class Emiel_PreparePenaltyThem : public Emiel_Prepare {
	public:
		Emiel_PreparePenaltyThem(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		void Initialize();
		Status Update();

		ros::NodeHandle nh;

		void initShootout();

	private:
		std::vector<boost::uuids::uuid> tokens;
		boost::optional<rtt::RefState> refState;
	};

}