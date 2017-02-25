#include <gtest/gtest.h>
#include "roboteam_tactics/skills/ParamSet.h"
#include "roboteam_tactics/conditions/ParamCheck.h"
#include "ros/ros.h"
#include <sstream>
#include <thread>

namespace rtt {
    
std::string paramName(int id) {
    std::ostringstream ss;
    ss << "/signal_" << id;
    return ss.str();
}

void producer(std::string param) {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetString("signal", param);
    bb->SetBool("value", true);
    ParamSet setter("", bb);
    bt::Node::Status ret = setter.Update();
    ASSERT_EQ(bt::Node::Status::Success, ret);
}

void consumer(std::string param) {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetString("signal", param);
    bb->SetBool("value", true);
    ParamCheck getter("", bb);
    int failCount = 0;
    bt::Node::Status status = bt::Node::Status::Invalid;
    while (failCount++ < 10 && status != bt::Node::Status::Success) {
        status = getter.Update();
    }
    ASSERT_EQ(bt::Node::Status::Success, status);
}

TEST(ParamsStressTest, stressTest) {
    int argc = 0;
    ros::init(argc, 0, "ParamsStressTest");
    ros::NodeHandle n;
    std::vector<std::thread*> producers, consumers;
    const int testCount = 1000;
    for (int i = 0; i < testCount; i++) {
        producers.push_back(new std::thread(&producer, paramName(1)));
        consumers.push_back(new std::thread(&consumer, paramName(1)));
    }
    /*for (int i = 0; i < testCount; i++) {
        producers[i]->detach();
        consumers[i]->detach();
    }*/
    for (int i = 0; i < testCount; i++) {
        producers[i]->join();
        delete producers[i];
        consumers[i]->join();
        delete consumers[i];
    }
}
    
}