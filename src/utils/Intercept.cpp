#include "roboteam_tactics/utils/Intercept.h"

namespace rtt {

#ifdef INTERCEPT_USE_MATLAB_GENERATED_CODE

#include "intercept/intercept2.h"

InterceptResult calculateInterception(InterceptQuery query) {
	double botPos[2] = {query.botPos.x, query.botPos.y};
	double botVel[2] = {query.botVel.x, query.botVel.y};
	double botAcc = query.botAcc;
	double ballPos[2] = {query.botPos.x, query.botPos.y};
	double ballVel[2] = {query.botPos.x, query.botPos.y};
	double ballAcc = query.ballAcc;

	double iPos[2];
	double iTime;
	unsigned char valid;

	intercept2(botPos, botVel, botAcc, ballPos, ballVel, ballAcc, &valid, iPos, &iTime);

	return {valid == 1, {iPos[0], iPos[1]}, iTime};
}

#else

inline bool isValid(const Vector2& iPos, double iTime) {
	return iPos.x >= -4.5 && iPos.x <= 4.5 && iPos.y >= -3 && iPos.y <= 3
			&& iTime > 0 && iTime < ICEPT_MAX_TIME && iPos.isNotNaN() && iTime == iTime;
}

std::pair<double, double> solve(InterceptQuery query, Vector2 far, double init,
		double max, double stepSize) {
	unsigned iterations = abs((max - init) / stepSize);
	double current = init;
	double bestResult = INFINITY;
	double bestArg = NAN;
	double bestDist = NAN;

	for (unsigned i = 0; i < iterations; i++) {
		Vector2 tgtPos = far * current + query.ballPos * (1 - current);
		Vector2 diff = tgtPos - query.botPos.location();
		double dist = diff.length();

		/*
		 * x = x0 + v*t + (a*t²) / 2
		 *
		 * Solve for t -->
		 *
		 * t = ( v + √(v² + 2*a*(x+x0) - 2*a*x0 ) ) / a
		 */

		// NOTE: This initial calculations omit the square root for speed.
		// Once a minimum is found, the calculation is repeated once with the square
		// root to obtain the correct time.

		double t = (query.botVel.x + query.botVel.x * query.botVel.x
				+ 2 * query.botAcc * (dist + query.botPos.x)
				- 2 * query.botAcc * query.botPos.x) / query.botAcc;

		// err = difference between the time it takes the robot to get to this point (t)
		// and the time it takes the ball (as a fraction of ICEPT_MAX_TIME).
	    double err = fabs(t - ICEPT_MAX_TIME * current);
		if (err < bestResult) {
			bestResult = err;
			bestArg = current;
			bestDist = dist;
			if (t < ICEPT_THRESHOLD) {
				return {current, (query.botVel.x + sqrt(query.botVel.x * query.botVel.x
						+ 2 * query.botAcc * (dist + query.botPos.x)
						- 2 * query.botAcc * query.botPos.x)) / query.botAcc};
			}
		}
		current += stepSize;
	}

	return {bestArg, (query.botVel.x + sqrt(query.botVel.x * query.botVel.x
			+ 2 * query.botAcc * (bestDist + query.botPos.x)
			- 2 * query.botAcc * query.botPos.x)) / query.botAcc};
}

InterceptResult calculateInterception(InterceptQuery query) {

	/*
	 * Strategy:
	 * 1. Find out where the ball will be after ICEPT_MAX_TIME seconds.
	 * 2. Find out which point along the ball's path the robot can reach the fastest.
	 *    2.1. We will traverse the ball's path in terms of a fraction of the total path,
	 *         such that 0 is the ball's current position and 1 is the position after ICEPT_MAX_TIME.
	 *    2.2  For each iteration, calculate the time by taking the general formula for calculating
	 *         distance travelled given an initial position, velocity, and acceleration in terms of time, and
	 *         rewriting it to yield the time instead, given the distance traveled.
	 *    2.3  See what fraction yields the (absolutely) smallest difference with the time it takes
	 *         the ball to get to the same point, and return that.
	 * 3. Check whether or not that point is real (non-NaN) and
	 *    within the bounds of the field. Also check whether it can be reached within ICEPT_MAX_TIME.
	 */

	Vector2 far { query.ballPos.x + query.ballVel.x * ICEPT_MAX_TIME,
			query.ballPos.y + query.ballVel.y * ICEPT_MAX_TIME };
	auto solution = solve(query, far, 0, 1, 0.01);
	double iFrac = solution.first;
	double iTime = solution.second;
	Vector2 iPos = (far - query.ballPos) * iFrac + query.ballPos;

	return {isValid(iPos, iTime), iPos, iTime};
}

#endif

}
