
    #include <iostream>
    #include <string>

    #include "ros/ros.h"

    #include "roboteam_tactics/bt.hpp"
    #include "roboteam_tactics/generated/alltrees.h"

    namespace rtt {

    bt::BehaviorTree make_tree(std::string name, bt::Blackboard* bb) {
        if (false) {
            // Bogus if clause
        } else if (name == "1_2_Tree_1") {
            return make_1_2_Tree_1(bb);
        }  else if (name == "1_2_Tree_2") {
            return make_1_2_Tree_2(bb);
        }  else if (name == "AttackerStrategy") {
            return make_AttackerStrategy(bb);
        }  else if (name == "BasicKeeperTree") {
            return make_BasicKeeperTree(bb);
        }  else if (name == "BasicRole") {
            return make_BasicRole(bb);
        }  else if (name == "BasicStrategy") {
            return make_BasicStrategy(bb);
        }  else if (name == "BounceBall") {
            return make_BounceBall(bb);
        }  else if (name == "CheckParamTest") {
            return make_CheckParamTest(bb);
        }  else if (name == "CoolRole") {
            return make_CoolRole(bb);
        }  else if (name == "CoolTree") {
            return make_CoolTree(bb);
        }  else if (name == "DemoStrategy") {
            return make_DemoStrategy(bb);
        }  else if (name == "DistanceTesterTree") {
            return make_DistanceTesterTree(bb);
        }  else if (name == "GoToPosTree") {
            return make_GoToPosTree(bb);
        }  else if (name == "SideSideStrategy") {
            return make_SideSideStrategy(bb);
        }  else if (name == "SideSideTestTree") {
            return make_SideSideTestTree(bb);
        }  else if (name == "SimpleSideSideStrategy") {
            return make_SimpleSideSideStrategy(bb);
        }  else if (name == "SuperCoolTree") {
            return make_SuperCoolTree(bb);
        }  else if (name == "TestBallInZone") {
            return make_TestBallInZone(bb);
        }  else if (name == "testparams") {
            return make_testparams(bb);
        } 
       
        std::cout << "Could not find tree with name " << name << ". Aborting\n";
        exit(1);
    }

    }