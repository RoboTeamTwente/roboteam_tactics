#include "roboteam_tactics/skills/ShootAway.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/debug_print.h"

#define RTT_CURRENT_DEBUG_TAG ShootAway

namespace rtt {

RTT_REGISTER_SKILL(ShootAway);

ShootAway::ShootAway(std::string name, bt::Blackboard::Ptr bb) : Skill(name, bb),
		 kicking(false), asapPushDecided(false), asapPush(false), asapPushAngle(-1.0),
		 highDecided(false), medDecided(false), lowDecided(false) {
}

bt::Node::Status ShootAway::Update() {
	if (kicking) {
		return kick->Update();
	}
	private_bb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
	std::string prio = GetString("priority", "HIGH");
	if (prio == "ASAP") return updateASAP();
	if (prio == "HIGH") return updateHIGH();
	if (prio == "MED") return updateMED();
	if (prio == "LOW") return updateLOW();
	ROS_ERROR("ShootAway: '%s' is not a valid priority! Defaulting to HIGH", prio.c_str());
	return updateHIGH();
}

Vector2 targetBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces) {
    Vector2 diff = ballPos - myPos;
    double theta = fabs(cleanAngle(diff.angle() - sumOfForces.angle()));

    if (theta < (0.5 * M_PI)) {
        if (fabs(theta) < .00001) theta = 0.01;
        // double force = theta / (0.5 * M_PI);
        Vector2 projectedBall = ballPos.project(myPos, myPos + sumOfForces);
        Vector2 ballForce = projectedBall - ballPos;
        sumOfForces = sumOfForces + ballForce * -15;
    }

    return sumOfForces;
}

bt::Node::Status ShootAway::updateASAP() {
	auto world = LastWorld::get();

	int robotID = blackboard->GetInt("ROBOT_ID");
	roboteam_msgs::WorldRobot bot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        bot = *findBot;
    } else {
        ROS_WARN("ShootAway could not find robot");
        return Status::Failure;
    }

	Vector2 botPos(bot.pos);
	Vector2 ballPos(world.ball.pos);
	double toBallAngle = (ballPos - botPos).angle();
	auto& pub = GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();

	if (!asapPushDecided) {
    	asapPushDecided = true;
    	double angleDiff = fabs(bot.angle - toBallAngle);
    	asapPush = angleDiff > ASAP_MAX_ROTATION;
    	if (asapPush) {
    		ROS_WARN("ShootAway: Robot %d is going to ram the ball (priority=ASAP)! Break out the spares!",
    				GetInt("ROBOT_ID"));
    		asapPushAngle = bot.angle;
    	} else {
    		private_bb->SetBool("ShootAway_ASAP_GetBall_passOn", true);
    		// No targetAngle / aimAt etc. We don't care where the ball ends up.
    		asapGetBall = std::make_unique<GetBall>(GetBall("ShootAway_ASAP_GetBall", private_bb));
    	}
    }

	if (asapPush) {
    	if (botPos.dist(ballPos) < .05) {
    		pub.publish(stop_command(GetInt("ROBOT_ID")));
    		return Status::Success;
    	}
    	Vector2 vel = (ballPos - botPos).stretchToLength(8.0);
    	double w = asapController.rotationController(bot.angle, asapPushAngle, Vector2());
    	vel = worldToRobotFrame(vel, bot.angle);
    	//vel = targetBall(ballPos, botPos, vel);
    	roboteam_msgs::RobotCommand cmd;
    	cmd.id = GetInt("ROBOT_ID");
    	cmd.x_vel = vel.x;
    	cmd.y_vel = vel.y;
    	cmd.dribbler = false; // We don't want the ball; we want it gone.
    	cmd.kicker = false;
    	cmd.w = w;
    	pub.publish(cmd);
    	return Status::Running;
    } else {
    	return asapGetBall->Update();
    }
}
bt::Node::Status ShootAway::updateHIGH() {
	RTT_DEBUGLN("Updating...");
	auto world = LastWorld::get();

	int robotID = blackboard->GetInt("ROBOT_ID");
	roboteam_msgs::WorldRobot bot;
	boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    if (findBot) {
        bot = *findBot;
    } else {
        ROS_WARN("ShootAway could not find robot");
        return Status::Failure;
    }

	Vector2 botPos(bot.pos);
	Vector2 ballPos(world.ball.pos);
	double toBallAngle = (ballPos - botPos).angle();

	if (!highDecided) {
		highDecided = true;
		double angleDiff = fabs(bot.angle - toBallAngle);
		if (angleDiff <= HIGH_MAX_ROTATION) {
			private_bb->SetString("ShootAway_HIGH_GetBall_aimAt", "theirgoal");
		}
		private_bb->SetBool("ShootAway_HIGH_GetBall_passOn", true);
		highGetBall = std::make_unique<GetBall>(GetBall("ShootAway_HIGH_GetBall", private_bb));
	}
	return highGetBall->Update();
}

std::vector<roboteam_msgs::WorldRobot> getOurBotsOnTheirSide() {
	auto world = LastWorld::get();
	std::vector<roboteam_msgs::WorldRobot> res;
	for (const auto& bot : world.us) {
		if (bot.pos.x >= 0) {
			res.push_back(bot);
		}
	}
	return res;
}

bt::Node::Status ShootAway::updateMED() {
	if (!medDecided) {
		medDecided = true;
		auto candidates = getOurBotsOnTheirSide();
		if (!candidates.empty()) {
			private_bb->SetString("ShootAway_MED_GetBall_aimAt", "robot");
			private_bb->SetBool("ShootAway_MED_GetBall_ourTeam", true);
			private_bb->SetInt("ShootAway_MED_GetBall_aimAtRobot", candidates.at(0).id);
		} else {
			private_bb->SetString("ShootAway_MED_GetBall_aimAt", "theirgoal");
		}
		private_bb->SetBool("ShootAway_MED_GetBall_passOn", true);
		medGetBall = std::make_unique<GetBall>(GetBall("ShootAway_MED_GetBall", private_bb));
	}
	return medGetBall->Update();
}

bt::Node::Status ShootAway::updateLOW() {
	if (!lowDecided) {
		lowDecided = true;
		roboteam_msgs::WorldRobot maxBot;
		for (const auto& bot : LastWorld::get().us) {
			if (bot.pos.x > maxBot.pos.x) {
				maxBot = bot;
			}
		}
		if (maxBot.pos.x > -.5) {
			private_bb->SetString("ShootAway_LOW_GetBall_aimAt", "robot");
			private_bb->SetBool("ShootAway_LOW_GetBall_ourTeam", true);
			private_bb->SetInt("ShootAway_LOW_GetBall_aimAtRobot", maxBot.id);
		} else {
			private_bb->SetString("ShootAway_LOW_GetBall_aimAt", "theirgoal");
		}
		private_bb->SetBool("ShootAway_LOW_GetBall_passOn", true);
		lowGetBall = std::make_unique<GetBall>("ShootAway_LOW_GetBall", private_bb);
	}
	return lowGetBall->Update();
}

}
