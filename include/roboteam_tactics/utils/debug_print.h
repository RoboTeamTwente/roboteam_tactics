#pragma once

#include <cstdio>

#define RTT_xstr(a) RTT_str(a)
#define RTT_str(a) #a
#define RTT_PASTER3(x, y, z) x ## _ ## y ## _ ## z

#define SET_DEBUG_FOR(tag, status) \
static const bool PRINT_ ## tag ## _MESSAGES = status

#define RTT_DEBUG_TAG(tag, format, ...) \
if (RTT_PASTER3(rtt::PRINT, tag, MESSAGES)) { \
    printf("[" RTT_xstr(tag) "] " format, ##__VA_ARGS__); \
}

#define RTT_DEBUG(format, ...) \
RTT_DEBUG_TAG( RTT_CURRENT_DEBUG_TAG , format, ##__VA_ARGS__)

#define RTT_DEBUGLN(format, ...) \
RTT_DEBUG(format "\n", ##__VA_ARGS__)

// Enable below here which debug statements should be on and/or off

namespace rtt {

// Top level nodes
SET_DEBUG_FOR(StrategyNode, true);
SET_DEBUG_FOR(RoleNode, true);

// Building blocks
SET_DEBUG_FOR(RobotDealer, true);
SET_DEBUG_FOR(ParallelTactic, false);
SET_DEBUG_FOR(Tactic, false);

// Skills
SET_DEBUG_FOR(AimAt, false);
SET_DEBUG_FOR(NaiveBlockGoal, false);
SET_DEBUG_FOR(ParamSet, false);
SET_DEBUG_FOR(GetBall, false);
SET_DEBUG_FOR(Kick, true);
SET_DEBUG_FOR(Failer, true);
SET_DEBUG_FOR(Runner, true);
SET_DEBUG_FOR(DefendGoalarea, true);

// Conditions
SET_DEBUG_FOR(ParamCheck, false);
SET_DEBUG_FOR(CanSeeTheirGoal, false);
SET_DEBUG_FOR(CanSeeRobot, false);
SET_DEBUG_FOR(DistanceXToY, false);
SET_DEBUG_FOR(IsRefStage, true);
SET_DEBUG_FOR(IsRefCommand, true);

// Tactics
SET_DEBUG_FOR(DemoTactic, true);
SET_DEBUG_FOR(AttackerTactic, true);
SET_DEBUG_FOR(BasicDefenseTactic, true);
SET_DEBUG_FOR(BasicKeeperTactic, true);

} // rtt
