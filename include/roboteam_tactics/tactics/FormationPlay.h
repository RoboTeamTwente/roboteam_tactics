#pragma once

#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/Position.h"
#include "roboteam_utils/json.hpp"

#include "ros/package.h"

#include <vector>
#include <map>
#include <string>

#define FORMATION_DEF_FILE ros::package::getPath("roboteam_tactics") + "/Formations.json"

namespace rtt {

struct Formation {
	std::vector<Position> positions;
	unsigned minimumRobots;
	std::string name;
	Formation(const nlohmann::json& json);
	Formation(const Formation& other);
	Formation();
};

class FormationPlay : public Tactic {
public:
	FormationPlay(std::string name, bt::Blackboard::Ptr bb);
	void Initialize() override;
	Status Update() override;
private:
	std::unique_ptr<Formation> formation;

	static bool formationsInitialized;
	static std::map<std::string, Formation> allFormations;
	static void initFormations();

};

}
