#include "roboteam_tactics/skills/SelectStarAttackShooter.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/BBParser.h"
#include "roboteam_tactics/utils/GoalPartition.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG SelectStarAttackShooter

namespace rtt {

constexpr std::chrono::nanoseconds SelectStarAttackShooter::MAX_WAIT;

RTT_REGISTER_SKILL(SelectStarAttackShooter);

SelectStarAttackShooter::SelectStarAttackShooter(std::string name, bt::Blackboard::Ptr bb)
	: Skill(name, bb) {
}

inline boost::optional<int> parseNum(const std::string& str) {
	for (auto c : str) {
		if (!std::isdigit(c)) return boost::none;
	}
	return atoi(str.c_str());
}

void SelectStarAttackShooter::Initialize() {
	if (!HasString("availableIDs")) {
		ROS_ERROR("SelectStarAttackShooter requires a string:availableIDs parameter");
		return;
	}
	std::string str = GetString("availableIDs");
	auto parts = split(str, ',');
	for (const std::string& part : parts) {
		if (part.size() == 0) continue;
		auto opt = parseNum(part);
		if (opt) {
			available.emplace(*opt);
		} else {
			ROS_WARN("SelectStarAttackShooter::Initialize: Not a valid ID: %s", part.c_str());
		}
	}
	start = now();
}

struct Sorter {
	bool operator()(const std::pair<int, Section>& a, const std::pair<int, Section>& b) {
		return a.second.length > b.second.length;
	}
};

boost::optional<std::pair<int, Section>> SelectStarAttackShooter::selectBest() const {
	std::vector<std::pair<int, Section>> sections;
	auto world = LastWorld::get();

	int robotID = blackboard->GetInt("ROBOT_ID");
	roboteam_msgs::WorldRobot robot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID); 
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN("SelectStarAttackShooter could not find robot");
        return boost::none;
    }

	Vector2 shooterPos(robot.pos);
	for (const auto& pair : largestOpenSections()) {
		auto bot = getWorldBot(pair.first);
		if (pair.second && bot &&
				getObstacles(*bot, shooterPos, &world, true, true).empty()) {
			sections.push_back({pair.first, *pair.second});
		}
	}
	if (sections.size() == 0) {
		return boost::none;
	}
	std::sort(sections.begin(), sections.end(), Sorter{});
	return sections.at(0);
}

bool SelectStarAttackShooter::hasBall() {

	int robotID = blackboard->GetInt("ROBOT_ID");
	roboteam_msgs::WorldRobot robot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN("SelectStarAttackShooter could not find robot");
        return false;
    }

	auto ball = LastWorld::get().ball;

	return bot_has_ball(robot, ball)	;
}

bt::Node::Status SelectStarAttackShooter::Update() {
	time_point current = now();

	if (!hasBall()) {
		return bt::Node::Status::Failure;
	}

	auto best = selectBest();
	if (best) {
		RTT_DEBUGLN("Best right now: %d@%f (%f, %f) -- time=%ld", best->first, best->second.length,
			best->second.a.y, best->second.b.y, (current - start).count());
	} else {
		RTT_DEBUGLN("None available right now...");
	}
	bool timeout = (current - start) > MAX_WAIT;
	if (timeout || (best && best->second.length >= MIN_DESIRED_GOAL_OPENING)) {
		if (!best) {
			ROS_WARN("SelectStarAttackShooter: Timeout expired, but no bot available");
			return bt::Node::Status::Running;
		}
		if (timeout) {
			ROS_WARN("SelectStarAttackShooter: Timeout expired, picking best available even if it wouldn't normally be good enough (%d).",
					best->first);
		} else {
			ROS_INFO("SelectStarAttackShooter: Found a good candidate: %d", best->first);
		}
		std::string param = GetString("idStoreParam");
		ros::param::set(param, best->first);
		ros::param::set("/signal_starRobotPicked", best->first);
		return bt::Node::Status::Success;
	} else {
		return bt::Node::Status::Running;
	}
}

std::map<int, boost::optional<Section>> SelectStarAttackShooter::largestOpenSections() const {
	std::map<int, boost::optional<Section>> result;
	for (int id : available) {
		GoalPartition partition;
		partition.calculatePartition(LastWorld::get(), Vector2(getWorldBot(id)->pos));
		result[id] = partition.largestOpenSection();
	}
	return result;
}


}
