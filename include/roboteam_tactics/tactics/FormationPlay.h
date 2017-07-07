#pragma once

#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/Position.h"
#include "roboteam_utils/json.hpp"

#include "ros/package.h"

#include <vector>
#include <map>
#include <string>

#define FORMATION_DEF_FILE ros::package::getPath("roboteam_tactics") + "/Formations.json"

/*
 * Coordinates in Formations.json should assume a field with the standard dimensions defined below.
 * All formations will be scaled according to the actual field size at runtime.
 * For example, on a field which is half as long and a third as wide as a standard field,
 * (2, 2) will become (1, 0.67) automatically.
 */

#define STANDARD_FIELD_DIMENSIONS Vector2 { 9.0, 6.0 }

namespace rtt {

struct Formation {
	std::vector<Position> positions;
	unsigned minimumRobots;
	std::string name;
	Formation(const nlohmann::json& json, Vector2 scaleFactors);
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
