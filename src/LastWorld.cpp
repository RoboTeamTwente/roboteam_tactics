#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"

#include "roboteam_tactics/LastWorld.h"

namespace rtt {

roboteam_msgs::World LastWorld::lastWorld;
roboteam_msgs::GeometryFieldSize LastWorld::field;
bool LastWorld::blue;

roboteam_msgs::World LastWorld::get() {
    return LastWorld::lastWorld;
}

void LastWorld::set(roboteam_msgs::World world) {
    LastWorld::lastWorld = world;
}

roboteam_msgs::GeometryFieldSize LastWorld::get_field() {
    return LastWorld::field;
}

void LastWorld::set_field(roboteam_msgs::GeometryFieldSize field) {
    LastWorld::field = field;
}

roboteam_msgs::Vector2f LastWorld::PredictBallPos(double t) {
	roboteam_msgs::WorldBall ball = lastWorld.ball;
	roboteam_msgs::Vector2f ballVel = ball.vel;
	roboteam_msgs::Vector2f predictedBallPos;
	predictedBallPos.x = ball.pos.x + ballVel.x * t;
	predictedBallPos.y = ball.pos.y + ballVel.y * t;
	return predictedBallPos;
}

bool LastWorld::we_are_blue() {
    return LastWorld::blue;
}

void LastWorld::set_we_are_blue(bool b) {
    LastWorld::blue = b;
}

}
