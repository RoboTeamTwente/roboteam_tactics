#pragma once

#include <cstdio>

namespace rtt {

void setStdoutToTeam();
void resetStdoutColor();

}

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

#define RTT_DEBUG_TEAM(format, ...) \
{ ::rtt::setStdoutToTeam(); RTT_DEBUG(format, ##__VA_ARGS__); ::rtt::resetStdoutColor(); }

#define RTT_DEBUGLN_TEAM(format, ...) \
{ ::rtt::setStdoutToTeam(); RTT_DEBUGLN(format, ##__VA_ARGS__); ::rtt::resetStdoutColor(); }

// Enable below here which debug statements should be on and/or off

namespace rtt {

// Top level nodes
SET_DEBUG_FOR(PracticeTest, true);
SET_DEBUG_FOR(RoleNode, true);
SET_DEBUG_FOR(StrategyNode, true);

// Building blocks
SET_DEBUG_FOR(ParallelTactic, true);
SET_DEBUG_FOR(RobotDealer, false);
SET_DEBUG_FOR(Tactic, false);

// Skills
SET_DEBUG_FOR(AimAt, false);
SET_DEBUG_FOR(Chip, true);
SET_DEBUG_FOR(DefendGoalarea, true);
SET_DEBUG_FOR(Failer, true);
SET_DEBUG_FOR(GetBall, false);
SET_DEBUG_FOR(GoToPos, true);
SET_DEBUG_FOR(InterceptBall, true);
SET_DEBUG_FOR(KeeperBlock, true);
SET_DEBUG_FOR(Kick, true);
SET_DEBUG_FOR(NaiveBlockGoal, false);
SET_DEBUG_FOR(ParamSet, false);
SET_DEBUG_FOR(QualKeeper, false);
SET_DEBUG_FOR(ReceiveBall, false);
SET_DEBUG_FOR(Runner, true);
SET_DEBUG_FOR(Harass, true);
SET_DEBUG_FOR(Wander, true);
SET_DEBUG_FOR(BlockGoal, true);
SET_DEBUG_FOR(SelectStarAttackShooter, true);
SET_DEBUG_FOR(ShootAway, true);

// Conditions
SET_DEBUG_FOR(CanSeeRobot, false);
SET_DEBUG_FOR(CanSeeTheirGoal, false);
SET_DEBUG_FOR(DistanceXToY, false);
SET_DEBUG_FOR(IsRefCommand, true);
SET_DEBUG_FOR(IsRefStage, true);
SET_DEBUG_FOR(ParamCheck, false);

// Tactics
SET_DEBUG_FOR(Anouk_MultipleDefendersPlay, true);
SET_DEBUG_FOR(AttackerTactic, true);
SET_DEBUG_FOR(BasicDefenseTactic, true);
SET_DEBUG_FOR(BasicKeeperTactic, true);
SET_DEBUG_FOR(DemoTactic, true);
SET_DEBUG_FOR(OneTwoTactic, true);
SET_DEBUG_FOR(PassToTactic, true);
SET_DEBUG_FOR(InterceptorsTactic, true);
SET_DEBUG_FOR(SecondaryKeeperTactic, true);
SET_DEBUG_FOR(FreeKickDefenceTactic, true);
SET_DEBUG_FOR(SoloAttackerTactic, true);
SET_DEBUG_FOR(SoloAttacker2Tactic, true);
SET_DEBUG_FOR(SoloDefenderTactic, true);
SET_DEBUG_FOR(StandByTactic, true);
SET_DEBUG_FOR(StarAttackTactic, true);
SET_DEBUG_FOR(TwoAttackersTactic, true);
SET_DEBUG_FOR(TwoAttackersCoolTactic, true);
SET_DEBUG_FOR(TwoVTwoDefenseTactic, true);
SET_DEBUG_FOR(PrepareKickoffUsTactic, true);
SET_DEBUG_FOR(KickoffUsTactic, true);
SET_DEBUG_FOR(Qualification1v1Tactic, true);
SET_DEBUG_FOR(WanderTactic, true);
SET_DEBUG_FOR(TwirlPlay, true);
SET_DEBUG_FOR(Bob_KickoffWithRunPlay, true);
SET_DEBUG_FOR(Bob_ChipoffAtGoalPlay, true);
SET_DEBUG_FOR(Jim_StandReadyPlay, true);
SET_DEBUG_FOR(Jim_MultipleStrikersPlay, true);
SET_DEBUG_FOR(Jim_MultipleDefendersPlay, true);
SET_DEBUG_FOR(Jim_MultipleDefendersPlayOld, true);
SET_DEBUG_FOR(BallPlacementThemPlay, true);
SET_DEBUG_FOR(BallPlacementUsPlay, true);
SET_DEBUG_FOR(Jim_GetBallPlay, true);
SET_DEBUG_FOR(Jim_IndirectGetBallPlay, true);
SET_DEBUG_FOR(Jim_TimeOut, true);
SET_DEBUG_FOR(Jim_KickOffDefense, true);
SET_DEBUG_FOR(Jim_PenaltyPlay, true);
SET_DEBUG_FOR(Jim_IndependentAttackersPlay, true);
SET_DEBUG_FOR(KickoffDefensePlay, true);
SET_DEBUG_FOR(StopPlay, true);
SET_DEBUG_FOR(Jelle_DemoPlay, true);
SET_DEBUG_FOR(Jelle_IndependentAttackersPlay, true);

// Other
SET_DEBUG_FOR(OpportunityFinder, true);
SET_DEBUG_FOR(Learner, true);
SET_DEBUG_FOR(DangerFinder, true);

} // rtt
