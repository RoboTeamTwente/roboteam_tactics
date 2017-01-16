#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/utils/ParallelTactic.hpp"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

    // Used skills: 
    #include "roboteam_tactics/skills/GoToPos.h"
    // Used conditions: 
    // Used tactics: 

    namespace rtt {

    namespace rtt_bob { 

    bt::BehaviorTree make_ProjectTest2(bt::Blackboard* blackboard) {
        bt::BehaviorTree tree;
        auto bb = tree.GetBlackboard();
        if(blackboard) merge_blackboards(bb, std::make_shared<bt::Blackboard>(*blackboard));
        auto GoToPos_A = std::make_shared<GoToPos>("GoToPos_A", bb);
        tree.SetRoot(GoToPos_A);
        return tree;
    }

    } /* rtt_bob */ 

    } // rtt

    namespace {

    rtt::factories::TreeRegisterer rtt_ProjectTest2_registerer("rtt_bob/ProjectTest2", &rtt::rtt_bob::make_ProjectTest2);

    } // anonymous namespace

    // Used skills: 
    #include "roboteam_tactics/skills/AimAt.h"
    #include "roboteam_tactics/skills/GetBall.h"
    #include "roboteam_tactics/skills/Kick.h"
    // Used conditions: 
    #include "roboteam_tactics/conditions/CanSeeRobot.h"
    #include "roboteam_tactics/conditions/CanSeeTheirGoal.h"
    // Used tactics: 

    namespace rtt {

    namespace rtt_bob { 

    bt::BehaviorTree make_BasicAttacker111(bt::Blackboard* blackboard) {
        bt::BehaviorTree tree;
        auto bb = tree.GetBlackboard();
        if(blackboard) merge_blackboards(bb, std::make_shared<bt::Blackboard>(*blackboard));
        auto Sequence_2 = std::make_shared<bt::Sequence>();
        auto GetBall_B = std::make_shared<GetBall>("GetBall_B", bb);
        GetBall_B->private_bb->SetString("getBallAtY", "");
        GetBall_B->private_bb->SetString("getBallAtTime", "");
        GetBall_B->private_bb->SetString("getBallAtX", "");
        auto Priority_3 = std::make_shared<bt::Selector>();
        auto Sequence_1 = std::make_shared<bt::Sequence>();
        auto CanSeeTheirGoal_B = std::make_shared<CanSeeTheirGoal>("CanSeeTheirGoal_B", bb);
        auto Kick_C = std::make_shared<Kick>("Kick_C", bb);
        auto Sequence_0 = std::make_shared<bt::Sequence>();
        auto CanSeeRobot_B = std::make_shared<CanSeeRobot>("CanSeeRobot_B", bb);
        auto AimAt_B = std::make_shared<AimAt>("AimAt_B", bb);
        AimAt_B->private_bb->SetString("At", "");
        auto Kick_B = std::make_shared<Kick>("Kick_B", bb);
        Sequence_2->AddChild(GetBall_B);
        Sequence_2->AddChild(Priority_3);
        Priority_3->AddChild(Sequence_1);
        Sequence_1->AddChild(CanSeeTheirGoal_B);
        Sequence_1->AddChild(Kick_C);
        Priority_3->AddChild(Sequence_0);
        Sequence_0->AddChild(CanSeeRobot_B);
        Sequence_0->AddChild(AimAt_B);
        Sequence_0->AddChild(Kick_B);
        tree.SetRoot(Sequence_2);
        return tree;
    }

    } /* rtt_bob */ 

    } // rtt

    namespace {

    rtt::factories::TreeRegisterer rtt_BasicAttacker111_registerer("rtt_bob/BasicAttacker111", &rtt::rtt_bob::make_BasicAttacker111);

    } // anonymous namespace

    // Used skills: 
    #include "roboteam_tactics/skills/AimAt.h"
    #include "roboteam_tactics/skills/GetBall.h"
    #include "roboteam_tactics/skills/Kick.h"
    // Used conditions: 
    #include "roboteam_tactics/conditions/DistanceXToY.h"
    // Used tactics: 

    namespace rtt {

    namespace rtt_bob { 

    bt::BehaviorTree make_BasicKeeperTree(bt::Blackboard* blackboard) {
        bt::BehaviorTree tree;
        auto bb = tree.GetBlackboard();
        if(blackboard) merge_blackboards(bb, std::make_shared<bt::Blackboard>(*blackboard));
        auto Repeat_1 = std::make_shared<bt::Repeater>();
        auto Priority_3 = std::make_shared<bt::Selector>();
        auto Parallel_A_2 = std::make_shared<bt::ParallelSequence>(2, 1);
        auto Sequence_4 = std::make_shared<bt::Sequence>();
        auto Distance_me_to_ball_A = std::make_shared<DistanceXToY>("Distance_me_to_ball_A", bb);
        Distance_me_to_ball_A->private_bb->SetString("mode", "lt");
        Distance_me_to_ball_A->private_bb->SetDouble("distance", 0.5);
        Distance_me_to_ball_A->private_bb->SetString("Y", "ball");
        Distance_me_to_ball_A->private_bb->SetString("X", "me");
        auto Distance_me_to_defense_area = std::make_shared<DistanceXToY>("Distance_me_to_defense_area", bb);
        Distance_me_to_defense_area->private_bb->SetString("mode", "lt");
        Distance_me_to_defense_area->private_bb->SetDouble("distance", 0.5);
        Distance_me_to_defense_area->private_bb->SetString("Y", "our defense area");
        Distance_me_to_defense_area->private_bb->SetString("X", "me");
        auto Sequence_5 = std::make_shared<bt::Sequence>();
        auto GetBall_A = std::make_shared<GetBall>("GetBall_A", bb);
        GetBall_A->private_bb->SetDouble("targetAngle", 3.14159);
        GetBall_A->private_bb->SetString("getBallAtY", "");
        GetBall_A->private_bb->SetString("getBallAtX", "");
        GetBall_A->private_bb->SetString("getBallAtTime", "");
        auto AimAt_A = std::make_shared<AimAt>("AimAt_A", bb);
        AimAt_A->private_bb->SetString("At", "");
        auto Kick_A = std::make_shared<Kick>("Kick_A", bb);
        Parallel_A_2->private_bb->SetDouble("minSuccess", 2);
        Parallel_A_2->private_bb->SetDouble("minFail", 1);
        auto Parallel_B_0 = std::make_shared<bt::ParallelSequence>(2, 1);
        auto Distance_me_to_ball_B = std::make_shared<DistanceXToY>("Distance_me_to_ball_B", bb);
        Distance_me_to_ball_B->private_bb->SetString("mode", "geq");
        Distance_me_to_ball_B->private_bb->SetDouble("distance", 0.5);
        Distance_me_to_ball_B->private_bb->SetString("Y", "ball");
        Distance_me_to_ball_B->private_bb->SetString("X", "me");
        auto GetBall_B = std::make_shared<GetBall>("GetBall_B", bb);
        GetBall_B->private_bb->SetString("getBallAtY", "");
        GetBall_B->private_bb->SetString("getBallAtTime", "");
        GetBall_B->private_bb->SetString("getBallAtX", "");
        Parallel_B_0->private_bb->SetDouble("minSuccess", 2);
        Parallel_B_0->private_bb->SetDouble("minFail", 1);
        Repeat_1->SetChild(Priority_3);
        Priority_3->AddChild(Parallel_A_2);
        Parallel_A_2->AddChild(Sequence_4);
        Sequence_4->AddChild(Distance_me_to_ball_A);
        Sequence_4->AddChild(Distance_me_to_defense_area);
        Parallel_A_2->AddChild(Sequence_5);
        Sequence_5->AddChild(GetBall_A);
        Sequence_5->AddChild(AimAt_A);
        Sequence_5->AddChild(Kick_A);
        Priority_3->AddChild(Parallel_B_0);
        Parallel_B_0->AddChild(Distance_me_to_ball_B);
        Parallel_B_0->AddChild(GetBall_B);
        tree.SetRoot(Repeat_1);
        return tree;
    }

    } /* rtt_bob */ 

    } // rtt

    namespace {

    rtt::factories::TreeRegisterer rtt_BasicKeeperTree_registerer("rtt_bob/BasicKeeperTree", &rtt::rtt_bob::make_BasicKeeperTree);

    } // anonymous namespace

