#include "roboteam_tactics/tactics/FormationPlay.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/LastWorld.h"
#include <fstream>

namespace rtt {

RTT_REGISTER_TACTIC(FormationPlay);

bool FormationPlay::formationsInitialized = false;
std::map<std::string, Formation> FormationPlay::allFormations;

inline Vector2 scaleFactors(Vector2 actualDimensions) {
	return actualDimensions / STANDARD_FIELD_DIMENSIONS;
}

void FormationPlay::initFormations() {
	if (formationsInitialized) return;
	formationsInitialized = true;

	auto field = LastWorld::get_field();
	Vector2 fieldDims { field.field_length, field.field_width };
	Vector2 scale = scaleFactors(fieldDims);

	std::ifstream fileStream(FORMATION_DEF_FILE);
	nlohmann::json allFormationDefs;
	allFormationDefs << fileStream;

	for (const nlohmann::json& json : allFormationDefs) {
		Formation f(json, scale);
		allFormations[f.name] = f;
	}
}

Formation::Formation(const nlohmann::json& json, Vector2 scaleFactors) {
	minimumRobots = json.at("minimumRobots");
	name = json.at("name");
	nlohmann::json::array_t posDefs = json.at("positions");
	for (const auto& def : posDefs) {
		// These casts are to get nlohmann::json's type inference to work properly.
		// Without them, gcc assumes def.at("x") is a std::complex...
		double x = static_cast<double>(def.at("x")) * scaleFactors.x;
		double y = static_cast<double>(def.at("y")) * scaleFactors.y;
		double rot = def.at("rot");
		positions.push_back({ x, y, rot });
	}
}

Formation::Formation(const Formation& other) : positions(other.positions),
		minimumRobots(other.minimumRobots), name(other.name) {}

Formation::Formation() : positions(), minimumRobots(0), name("<not initialized>") {}

FormationPlay::FormationPlay(std::string name, bt::Blackboard::Ptr bb) : Tactic(name, bb) {}

void FormationPlay::Initialize() {
	initFormations();
	if (!HasString("formation")) {
		ROS_ERROR("FormationPlay: No string 'formation' in the blackboard!");
		return;
	}
	std::string name = GetString("formation");
	if (allFormations.find(name) == allFormations.end()) {
		ROS_ERROR("FormationPlay: Formation '%s' was not found", name.c_str());
	}

	formation = std::make_unique<Formation>(allFormations.at(name));
	std::vector<int> robots = RobotDealer::get_available_robots();
	if (robots.size() < formation->minimumRobots) {
		ROS_WARN("FormationPlay: Not enough robots available, continuing anyway");
	}
	unsigned count = std::min(robots.size(), formation->positions.size());

	auto& pub = rtt::GlobalPublisher<roboteam_msgs::RoleDirective>::get_publisher();

	for (unsigned i = 0; i < count; i++) {
		int id = robots.at(i);
		claim_robot(id);
		Position pos = formation->positions.at(i);
		roboteam_msgs::RoleDirective rd;
		rd.robot_id = id;
		rd.tree = "rtt_dennis/KeepPositionRole";
		bt::Blackboard bb;
		bb.SetInt("ROBOT_ID", id);
		bb.SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		bb.SetDouble("GoToPos_A_xGoal", pos.x);
		bb.SetDouble("GoToPos_A_yGoal", pos.y);
		bb.SetDouble("GoToPos_A_angleGoal", pos.rot);
		bb.SetDouble("GoToPos_A_maxVelocity", STOP_STATE_MAX_VELOCITY);
		rd.blackboard = bb.toMsg();
		pub.publish(rd);

        std::cout << "Sending robot " << id << " to " << Vector2(pos.x, pos.y) << "\n";
	}
}

bt::Node::Status FormationPlay::Update() {
	return Status::Running;
}

}
