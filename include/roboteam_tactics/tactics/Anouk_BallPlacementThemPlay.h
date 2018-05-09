#pragma once

#include <chrono>

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

/**
 * Max distance ball can be from placement point in meters.
 */
//double const MAX_DISTANCE_FROM_END_POINT = 0.1;

class Anouk_BallPlacementThemPlay : public Tactic {
public:
    Anouk_BallPlacementThemPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

	void Initialize() override;
    Status Update() override;
    
    std::string node_name() override { return "Anouk_BallPlacementThemPlay"; }

private:
    boost::uuids::uuid token;
    bool failed;
    bool succeeded;

	Vector2 lastBallPlacementPosition;
	int placerID;
	long int timeBallPlaced;

	void movePlacerAwayFromBall();
	bool ballPlacementSuccessful();

	enum PlayStates {
		PLACING_BALL,
		STOP_BALL_SPINNING,
		MOVING_PLACER
	};
	PlayStates currentState;



} ;

} // rtt


