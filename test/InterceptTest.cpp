#include "../src/utils/intercept/intercept2.h"
#include "roboteam_tactics/utils/utils.h"
#include <iostream>
#include <gtest/gtest.h>
#include <chrono>
#include <vector>

TEST(InterceptTest, InterceptTest) {

	std::vector<double> posX, posY, velX, velY, bPosX, bPosY, bVelX, bVelY;
	unsigned iterations = 10000;

	for (unsigned i = 0; i < iterations; i++) {
		posX.push_back(rtt::get_rand_real(-9, 9));
		posY.push_back(rtt::get_rand_real(-9, 9));
		velX.push_back(rtt::get_rand_real(-4, 4));
		velY.push_back(rtt::get_rand_real(-4, 4));
		bPosX.push_back(rtt::get_rand_real(-9, 9));
		bPosY.push_back(rtt::get_rand_real(-9, 9));
		bVelX.push_back(rtt::get_rand_real(-4, 4));
		bVelY.push_back(rtt::get_rand_real(-4, 4));
	}

	unsigned success = 0;
	unsigned nans = 0;
	auto start = std::chrono::steady_clock::now();

	for (unsigned i = 0; i < iterations; i++) {
		double botPos[2] = { posX[i], posY[i] };
		double botVel[2] = { velX[i], velY[i] };
		double botAcc = 3;

		double ballPos[2] = { bPosX[i], bPosY[i] };
		double ballVel[2] = { bVelX[i], bVelY[i] };
		double ballAcc = 0;

		double iPos[2];
		double iTime;
		unsigned char valid;

		intercept2(botPos, botVel, botAcc, ballPos, ballVel, ballAcc, &valid, iPos, &iTime);
		if (valid) success++;
		if (iPos[0] != iPos[0] || iPos[1] != iPos[1] || iTime != iTime) nans++;
		//std::cout << std::boolalpha << "Results:\n\tvalid = " << valid << "\n\tiPos = ("
		// << iPos[0] << ", " << iPos[1] << "\n\tiTime = " << iTime << "\n";
	}

	auto end = std::chrono::steady_clock::now();
	std::cout << "Time: " << (end - start).count() / 1000 << "us (avg "
			<< (end - start).count() / (1000 * iterations) << " us) Success Count: " << success
			<< " NaN Count: " << nans << "\n";

}
