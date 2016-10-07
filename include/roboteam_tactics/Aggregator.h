#pragma once

#include <memory>
#include <unordered_map>

#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/SteeringAction.h"

class Aggregator {
    public:
        actionlib::SimpleActionClient<roboteam_msgs::SteeringAction>* getClient(int i);
        void putMsg(int robotID, const roboteam_msgs::SteeringGoal& goal);
        actionlib::SimpleClientGoalState getState(int it);

    private:
        std::unordered_map<int, std::unique_ptr<actionlib::SimpleActionClient<roboteam_msgs::SteeringAction>>> clients;
} ;

