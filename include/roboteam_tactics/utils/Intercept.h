#pragma once

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

// Uncomment this define to use the matlab-generated interception code.
//#define INTERCEPT_USE_MATLAB_GENERATED_CODE

namespace rtt {

struct InterceptResult {
	bool success;
	Vector2 pos;
	double time;
};

struct InterceptQuery {
	Position botPos;
	Vector2 botVel;
	double botAcc;

	Vector2 ballPos;
	Vector2 ballVel;
	double ballAcc;
};

constexpr double ICEPT_BOT_ACC { 3.0 };
constexpr double ICEPT_BALL_ACC { 0.0 }; // TODO?
constexpr double ICEPT_MAX_TIME { 3.0 }; // seconds
constexpr double ICEPT_THRESHOLD { 0.01 };
constexpr InterceptQuery buildNormalInterceptQuery(Position botPos, Vector2 botVel, Vector2 ballPos, Vector2 ballVel) {
	return { botPos, botVel, ICEPT_BOT_ACC, ballPos, ballVel, ICEPT_BALL_ACC };
}

constexpr InterceptQuery defaultInterceptQuery = buildNormalInterceptQuery({0, 0, 0},
		{0, 0}, {1, 1}, {0, -1});


InterceptResult calculateInterception(InterceptQuery query);

}


