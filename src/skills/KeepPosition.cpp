#include "roboteam_tactics/skills/KeepPosition.h"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

RTT_REGISTER_SKILL (KeepPosition);

KeepPosition::KeepPosition(std::string name, bt::Blackboard::Ptr bb) :
		Skill(name, bb) {
}

std::pair<boost::optional<Vector2>, boost::optional<Vector2>> circleIntersection(
		Vector2 p1, double r1, Vector2 p2, double r2) {
	double dist = p1.dist(p2);
	if (dist > r1 + r2) {
		// Too far apart
		return {boost::none, boost::none};
	} else if (dist < fabs(r1 - r2)) {
		ROS_WARN(
				"KeepPosition.cpp:circleIntersection - One circle is contained in another, this seems unlikely");
		return {boost::none, boost::none};
	} else if (dist < .001 && fabs(r1 - r2) < .001) {
		// These circles are the same...
		return {boost::none, boost::none};
	} else {
		double a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
		double h = sqrt(r1 * r1 - a * a);
		Vector2 intBase { p1.x + a * (p2.x - p1.x) / dist, p1.y
				+ a * (p2.y - p1.y) / dist };
		Vector2 intA { intBase.x + h * (p2.y - p1.y) / dist, intBase.y
				- h * (p2.x - p1.x) / dist };

		if (fabs(dist - r1 + r2) < .001) {
			// Single 'intersection'
			return {intA, boost::none};
		}

		Vector2 intB { intBase.x - h * (p2.y - p1.y) / dist, intBase.y
				+ h * (p2.x - p1.x) / dist };

		return {intA, intB};
	}
}

bt::Node::Status KeepPosition::Update() {
	if (!gtp) {
		const auto bot = getWorldBot(GetInt("ROBOT_ID"));
		if (!bot) {
			ROS_ERROR("KeepPosition: Bot with ROBOT_ID (=%d) not found...",
					GetInt("ROBOT_ID"));
			return Status::Invalid;
		}
		if (GetBool("returnToInitialPos", false)) {
			initialPos = Position(*bot);
		}
		private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
		private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		private_bb->SetDouble("KeepPosition_GTP_targetAngle", bot->angle);
		private_bb->SetBool("KeepPosition_GTP_avoidRobots", true);
		private_bb->SetBool("KeepPosition_GTP_avoidBall", true);
		gtp = std::make_unique<GoToPos>("KeepPosition_GTP", private_bb);
	}
	if (!updateGoalPosition()) {
		ROS_ERROR("UpdateGoalPosition failed!\n");
		return Status::Invalid;
	}
	auto subStatus = gtp->Update();
	if (subStatus == Status::Invalid || subStatus == Status::Failure) {
		ROS_ERROR("Substatus omg!\n");
		return subStatus;
	}
	return Status::Running;
}

bool KeepPosition::updateGoalPosition() {
	const auto bot = getWorldBot(GetInt("ROBOT_ID"));
	if (!bot) {
		ROS_ERROR("KeepPosition: Bot with ROBOT_ID (=%d) not found...",
				GetInt("ROBOT_ID"));
		return false;
	}

	Vector2 ownPos { bot->pos };
	Vector2 referencePos = initialPos ? initialPos->location() : ownPos;
	Vector2 ballPos { LastWorld::get().ball.pos };

	std::vector<Vector2> obstacles;

	auto bots = LastWorld::get().us;
	auto them = LastWorld::get().them;
	bots.insert(bots.end(), them.begin(), them.end());

	if (ownPos.dist(ballPos) < MINIMUM_ROBOT_DISTANCE) {
		obstacles.push_back(ballPos);
	}
	for (const auto& bot : bots) {
		Vector2 pos { bot.pos };
		if (pos != ownPos && pos.dist(ownPos) < MINIMUM_ROBOT_DISTANCE) {
			obstacles.push_back(pos);
		}
	}

	Vector2 goal;
	if (obstacles.size() == 0) {
		goal = referencePos;
	} else if (obstacles.size() == 1) {
		Vector2 diff = obstacles.at(0) - ownPos;
		Vector2 desired_distance = diff.stretchToLength(
				MINIMUM_ROBOT_DISTANCE - diff.length());
		goal = desired_distance.rotate(M_PI) + ownPos;
	} else {
		std::vector<Vector2> candidates;
		for (unsigned i = 0; i < obstacles.size(); i++) {
			for (unsigned j = 0; j < i; j++) {
				std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections =
						circleIntersection(obstacles.at(i),
								MINIMUM_ROBOT_DISTANCE, obstacles.at(j),
								MINIMUM_ROBOT_DISTANCE);
				if (intersections.first) {
					bool clear = true;
					for (const Vector2& obs : obstacles) {
						if (intersections.first->dist(obs)
								< MINIMUM_ROBOT_DISTANCE) {
							clear = false;
							break;
						}
					}
					if (clear) {
						candidates.push_back(*intersections.first);
					}
				}
				if (intersections.second) {
					bool clear = true;
					for (const Vector2& obs : obstacles) {
						if (intersections.second->dist(obs)
								< MINIMUM_ROBOT_DISTANCE) {
							clear = false;
							break;
						}
					}
					if (clear) {
						candidates.push_back(*intersections.second);
					}
				}
			}
		}
		goal = candidates.size() > 0 ? candidates.at(0) : ownPos;
	}

	private_bb->SetDouble("KeepPosition_GTP_xGoal", goal.x);
	private_bb->SetDouble("KeepPosition_GTP_yGoal", goal.y);
	private_bb->SetDouble("KeepPosition_GTP_angleGoal",
			initialPos ? initialPos->rot : bot->angle);
	private_bb->SetBool("KeepPosition_GTP_avoidRobots", true);
	private_bb->SetBool("KeepPosition_GTP_avoidBall", true);
	private_bb->SetDouble("KeepPosition_GTP_maxVelocity",
			STOP_STATE_MAX_VELOCITY);
	return true;
}

struct DistToPosSorter {
	Vector2 ownPos;
	bool operator()(const Vector2& a, const Vector2& b) {
		return a.dist2(ownPos) < b.dist2(ownPos);
	}
	bool operator()(const roboteam_msgs::WorldRobot& a,
			const roboteam_msgs::WorldRobot& b) {
		return Vector2 { a.pos }.dist2(ownPos) < Vector2 { b.pos }.dist2(ownPos);
	}
};

Vector2 KeepPosition::getNearestObject(Vector2 ownPos) {
	const auto& world = LastWorld::get();
	auto us = world.us;
	auto them = world.them;

	bool opponentsExist = them.size() > 0;

	us.erase(
			std::remove_if(us.begin(), us.end(),
					[=](const roboteam_msgs::WorldRobot& bot) {return bot.id == (unsigned) this->GetInt("ROBOT_ID");}),
			us.end());

	std::sort(us.begin(), us.end(), DistToPosSorter { ownPos });
	if (opponentsExist) {
		std::sort(them.begin(), them.end(), DistToPosSorter { ownPos });

		Vector2 closestUs { us.at(0).pos };
		Vector2 closestThem { them.at(0).pos };
		Vector2 ball { world.ball.pos };

		std::vector<Vector2> v { closestUs, closestThem, ball };
		std::sort(v.begin(), v.end(), DistToPosSorter { ownPos });

		return v.at(0);
	} else {
		Vector2 closest { us.at(0).pos };
		Vector2 ball { world.ball.pos };
		return closest.dist2(ownPos) < ball.dist2(ownPos) ? closest : ball;
	}
}

}
