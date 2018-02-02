#include "roboteam_tactics/skills/Evade.h"
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {

RTT_REGISTER_SKILL (Evade);

Evade::Evade(std::string name, bt::Blackboard::Ptr bb) :
		Skill(name, bb) {
}

bt::Node::Status Evade::Update() {
	if (!gtp) {
		const auto bot = getWorldBot(GetInt("ROBOT_ID"));
		if (!bot) {
			ROS_ERROR("Evade: Bot with ROBOT_ID (=%d) not found...",
					GetInt("ROBOT_ID"));
			return Status::Invalid;
		}
		if (GetBool("returnToInitialPos", false)) {
			initialPos = Position(*bot);
		}
		private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
		private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
		private_bb->SetDouble("Evade", bot->angle);
		private_bb->SetBool("Evade_GTP_avoidRobots", true);
		private_bb->SetBool("Evade_GTP_avoidBall", true);
		private_bb->SetBool("Evade_GTP_enterDefenseAreas", true);
		gtp = std::make_unique<GoToPos>("Evade_GTP", private_bb);
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

bool Evade::updateGoalPosition() {
	const auto bot = getWorldBot(GetInt("ROBOT_ID"));
	if (!bot) {
		ROS_ERROR("Evade: Bot with ROBOT_ID (=%d) not found...",
				GetInt("ROBOT_ID"));
		return false;
	}



	Vector2 ownPos { bot->pos };
	// Vector2 referencePos = initialPos ? initialPos->location() : ownPos;
	Vector2 ballPos { LastWorld::get().ball.pos };

	auto bots = LastWorld::get().us;
	auto them = LastWorld::get().them;
	bots.insert(bots.end(), them.begin(), them.end());

	unsigned ownId { bot->id };
	Vector2 maxVel = Vector2(0,0);
	Vector2 relevantMaxVel = Vector2(0,0);
	float distance;
	for (const auto& bot : bots) {
		Vector2 pos { bot.pos };
		Vector2 vel { bot.vel };
		unsigned id { bot.id };
		float dist = pos.dist(ownPos);

		if (id != ownId && dist < LOOKING_DISTANCE && vel.length()>0.01) {
			Vector2 relevantVel = vel.project2(ownPos - pos);
			if(relevantVel.length() > 0.05 && relevantVel.length() > relevantMaxVel.length() && relevantVel.dot(ownPos - pos) > 0) { //TWEAK: threshold for seeing moving robot
				maxVel = vel;
				relevantMaxVel = relevantVel;
				distance = dist;
			}
		}
	}
	
	Vector2 goal = ownPos; //goal remains ownPos if no moving object in sight
	if (relevantMaxVel.length() > 0.05) { //TWEAK: threshold for seeing moving robot
		float angle = maxVel.angle() - relevantMaxVel.angle();
		if(angle > M_PI) {
			angle -= 2 * M_PI;
		} else if (angle < -M_PI) {
			angle += 2 * M_PI;
		}
		Vector2 rotatedVector = maxVel.rotate((angle < 0 ? .5 : -.5) * M_PI); //TWEAK: maxVel.scale(..)
		double scaling = distance / LOOKING_DISTANCE;
		Vector2 goalVector = relevantMaxVel.lerp(rotatedVector, 0.5*(1.0-scaling)); //TWEAK: less effect for relevantMaxVel: ...*(1.0-scaling)
		// drawer.setColor(255, 0, 0);
  //   	drawer.drawLine("goalVector", ownPos, goalVector);
		goal = ownPos + goalVector.scale(0.1/goalVector.length() + 0.1/distance/distance); //TWEAK: scale(....) //TWEAK: stretchToLength instead of scale, to remove speed dependency
	}

	private_bb->SetDouble("Evade_GTP_xGoal", goal.x);
	private_bb->SetDouble("Evade_GTP_yGoal", goal.y);
	// private_bb->SetDouble("Evade_GTP_angleGoal",
			// initialPos ? initialPos->rot : bot->angle);
	private_bb->SetBool("Evade_GTP_avoidRobots", true);
	private_bb->SetBool("Evade_GTP_avoidBall", true);
	private_bb->SetBool("Evade_GTP_enterDefenseAreas", true);
	private_bb->SetDouble("Evade_GTP_maxVelocity",
			STOP_STATE_MAX_VELOCITY);
	return true;
}

}
