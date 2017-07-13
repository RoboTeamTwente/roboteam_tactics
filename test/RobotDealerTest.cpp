#include <gtest/gtest.h>
#include <algorithm>
#include "roboteam_tactics/utils/RobotDealer.h"

void sharedInit() {
	// Initialize and check
	rtt::RobotDealer::initialize_robots(0, {1,2,3,4,5});
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());
	ASSERT_EQ(0, rtt::RobotDealer::get_keeper());
	ASSERT_TRUE(rtt::RobotDealer::get_keeper_available());
}

TEST(RobotDealerTest, goodTests) {
	sharedInit();

	// Simple, correct claim and release of the keeper
	ASSERT_TRUE(rtt::RobotDealer::claim_robot(0));
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());
	ASSERT_FALSE(rtt::RobotDealer::get_keeper_available());
	ASSERT_TRUE(rtt::RobotDealer::release_robot(0));
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());

	// Simple, correct claim and release of a non-keeper robot
	ASSERT_TRUE(rtt::RobotDealer::claim_robot(1));
    ASSERT_EQ(4, rtt::RobotDealer::get_available_robots().size());
    ASSERT_TRUE(rtt::RobotDealer::get_keeper_available());
    ASSERT_TRUE(rtt::RobotDealer::release_robot(1));
    ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());

    // Correct multi-robot claim and release
	ASSERT_TRUE(rtt::RobotDealer::claim_robots({1,2,3}));
	auto avail = rtt::RobotDealer::get_available_robots();
	ASSERT_EQ(2, avail.size());
	ASSERT_EQ(4, avail.at(0));
	ASSERT_EQ(5, avail.at(1));
	ASSERT_TRUE(rtt::RobotDealer::get_keeper_available());
	ASSERT_TRUE(rtt::RobotDealer::release_robots({1,2,3}));
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());

	// Check final state
	avail = rtt::RobotDealer::get_available_robots();
	ASSERT_TRUE(avail == std::vector<int>({1,2,3,4,5}));
}

TEST(RobotDealerTest, badTests_HERE_BE_ERRORS) {
	sharedInit();

	// Release a non-existent robot
	ASSERT_FALSE(rtt::RobotDealer::release_robot(9)); // prints a ROS_ERROR
	// Check that the list of available robots has not been impacted
	auto avail = rtt::RobotDealer::get_available_robots();
	ASSERT_EQ(5, avail.size());
	ASSERT_TRUE(std::find(avail.begin(), avail.end(), 9) == avail.end());

	// Double-claim a robot
	ASSERT_TRUE(rtt::RobotDealer::claim_robot(1));
	ASSERT_FALSE(rtt::RobotDealer::claim_robot(1)); // prints a ROS_ERROR
	avail = rtt::RobotDealer::get_available_robots();
	ASSERT_EQ(4, avail.size()); // Check that the robot is still claimed

	// Double-release a robot
	ASSERT_TRUE(rtt::RobotDealer::release_robot(1));
	ASSERT_FALSE(rtt::RobotDealer::release_robot(1)); // prints a ROS_ERROR
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());

	// Claim a non-existent robot
	ASSERT_FALSE(rtt::RobotDealer::claim_robot(9)); // prints a ROS_ERROR
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size());

	// Claim a robot, then include that same robot in a multi-claim
	ASSERT_TRUE(rtt::RobotDealer::claim_robot(3)); // correct
	ASSERT_FALSE(rtt::RobotDealer::claim_robots({2,3,4})); // incorrect, prints a ROS_ERROR
	avail = rtt::RobotDealer::get_available_robots();
	ASSERT_EQ(2, avail.size());
	ASSERT_EQ(1, avail.at(0));
	ASSERT_EQ(5, avail.at(1));

	// Release an unclaimed robot (1) in a multi-release
	ASSERT_FALSE(rtt::RobotDealer::release_robots({1,2,3,4})); // prints a ROS_ERROR
	ASSERT_EQ(5, rtt::RobotDealer::get_available_robots().size()); // the other robots should have been released

	// Check final state
	avail = rtt::RobotDealer::get_available_robots();
	ASSERT_TRUE(avail == std::vector<int>({1,2,3,4,5}));
}
