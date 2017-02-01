#pragma once

#include <cstdio>

#define RTT_xstr(a) RTT_str(a)
#define RTT_str(a) #a
#define RTT_PASTER3(x, y, z) x ## _ ## y ## _ ## z

#define SET_DEBUG_FOR(tag, status) \
static const bool PRINT_ ## tag ## _MESSAGES = status

#define RTT_DEBUG_TAG(tag, format, ...) \
if (RTT_PASTER3(::rtt::PRINT, tag, MESSAGES)) { \
    printf("[" RTT_xstr(tag) "] " format, ##__VA_ARGS__); \
}

#define RTT_DEBUG(format, ...) \
RTT_DEBUG_TAG( RTT_CURRENT_DEBUG_TAG , format, ##__VA_ARGS__)

#define RTT_DEBUGLN(format, ...) \
RTT_DEBUG(format "\n", ##__VA_ARGS__)

// Enable below here which debug statements should be on and/or off

namespace rtt {

// Top level nodes
SET_DEBUG_FOR(PracticeTest, true);
SET_DEBUG_FOR(RoleNode, true);
SET_DEBUG_FOR(StrategyNode, true);

// Building blocks
SET_DEBUG_FOR(ParallelTactic, false);
SET_DEBUG_FOR(RobotDealer, false);
SET_DEBUG_FOR(Tactic, false);

// Skills
SET_DEBUG_FOR(AimAt, false);
SET_DEBUG_FOR(Chip, true);
SET_DEBUG_FOR(DefendGoalarea, true);
SET_DEBUG_FOR(Failer, true);
SET_DEBUG_FOR(GetBall, true);
SET_DEBUG_FOR(InterceptBall, true);
SET_DEBUG_FOR(Kick, false);
SET_DEBUG_FOR(NaiveBlockGoal, false);
SET_DEBUG_FOR(ParamSet, false);
SET_DEBUG_FOR(ReceiveBall, true);
SET_DEBUG_FOR(Runner, true);

// Conditions
SET_DEBUG_FOR(CanSeeRobot, false);
SET_DEBUG_FOR(CanSeeTheirGoal, false);
SET_DEBUG_FOR(DistanceXToY, false);
SET_DEBUG_FOR(IsRefCommand, true);
SET_DEBUG_FOR(IsRefStage, true);
SET_DEBUG_FOR(ParamCheck, false);

// Tactics
SET_DEBUG_FOR(AttackerTactic, true);
SET_DEBUG_FOR(BasicDefenseTactic, true);
SET_DEBUG_FOR(BasicKeeperTactic, true);
SET_DEBUG_FOR(DemoTactic, true);
SET_DEBUG_FOR(OneTwoTactic, true);
SET_DEBUG_FOR(PassToTactic, true);
SET_DEBUG_FOR(SoloAttackerTactic, true);
SET_DEBUG_FOR(SoloDefenderTactic, true);

// Other
SET_DEBUG_FOR(ComputePassPoint, true);
SET_DEBUG_FOR(Learner, true);


} // rtt
