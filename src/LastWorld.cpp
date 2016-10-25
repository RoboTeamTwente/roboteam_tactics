#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"

#include "roboteam_tactics/LastWorld.h"

namespace rtt {

roboteam_msgs::World LastWorld::lastWorld;


void LastWorld::set(roboteam_msgs::World world) {
    LastWorld::lastWorld = world;
}

roboteam_msgs::Vector2f LastWorld::PredictBallPos(double t) {
	roboteam_msgs::WorldBall ball = lastWorld.ball;
	roboteam_msgs::Vector2f ballVel = ball.vel;
	roboteam_msgs::Vector2f predictedBallPos;
	predictedBallPos.x = ball.pos.x + ballVel.x * t;
	predictedBallPos.y = ball.pos.y + ballVel.y * t;
	return predictedBallPos;
}

roboteam_msgs::World LastWorld::get() {
    return LastWorld::lastWorld;
}

}