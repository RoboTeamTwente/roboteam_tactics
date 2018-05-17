#pragma once

#include "roboteam_tactics/Parts.h"
#include <boost/uuid/uuid.hpp>

namespace rtt {

	class Emiel_Prepare : public Tactic {
	public:
		Emiel_Prepare(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

		std::vector <boost::uuids::uuid> prepare(std::vector<Vector2> positions);
		std::vector <boost::uuids::uuid> prepare(std::vector<Vector2> positions, std::vector<int> robotsToDefend);

		ros::NodeHandle nh;
	private:

		boost::uuids::uuid init_ballDefender(int robotID, Vector2 position);
		boost::uuids::uuid init_robotDefender(int robotID, int opponentID);
		bool init_robotDefender();

	};
}