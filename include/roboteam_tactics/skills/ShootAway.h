#pragma once

#include "roboteam_tactics/Parts.h"
#include "Kick.h"
#include "GetBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/utils/Control.h"

namespace rtt {

 //TODO: Retrofit to use chipping once that's available!

 //TODO: ASAP does not work well yet; it needs better control (custom or GoToPos)

/*
 * Descr: |
 *   Gets the ball away from where it is now, not caring much about where it ends up.
 *   This skill is meant to be a sort of emergency option to prevent an imminent goal.
 *   There are a few different modes for this skill, which strike a different balance between
 *   getting the ball away quickly and getting it to a favorable position.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The robot which is to shoot
 *   - priority:
 *       Type: String
 *       Descr: The mode to run in.
 *       Can be:
 *         - ASAP: |
 *             Kick or push the ball out of play as soon as possible with no regard for the consequences.
 *             This is VERY dangerous; the robot might well drive itself into the wall at high speed.
 *             For emergency use only.
 *         - HIGH: If it's easy, then try to kick the ball to the opponents' side or bounce it of an opponent
 *         - MED: Always kick the ball towards the opponents' side, try to get it to one of our robots
 *         - LOW: Kick the ball towards whichever of our robots is the closest to the opponents' goal
 *       Default: HIGH
 */
class ShootAway : public Skill {
public:
	ShootAway(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update() override;
private:
	static constexpr double ASAP_MAX_ROTATION = M_PI_4l; //< Max. angle in radians which the ASAP priority is willing to
	                                                     //  turn. If the robot would have to turn more than this, it will
	                                                     //  simply ram the ball.

	static constexpr double HIGH_MAX_ROTATION = M_PI_2l; //< Max. angle in radians which the HIGH priority is willing to
														 //  turn. If the robot would have to turn more than this, it will
    													 //  kick the ball in whatever direction it can instead of aiming
	                                                     //  towards the opponents.

	bool kicking;
	std::unique_ptr<Kick> kick;

	bool asapPushDecided;
	bool asapPush;
	Vector2 asapGoal;
	double asapPushAngle;
	Control asapController;
    std::unique_ptr<GetBall> asapGetBall;

    bool highDecided;
    std::unique_ptr<GetBall> highGetBall;

    bool medDecided;
    std::unique_ptr<GetBall> medGetBall;

    bool lowDecided;
    std::unique_ptr<GetBall> lowGetBall;

	Status updateASAP();
	Status updateHIGH();
	Status updateMED();
	Status updateLOW();


};

}

