#include <memory>
#include <unordered_map>

#include "actionlib/client/simple_action_client.h"

#include "roboteam_msgs/SteeringAction.h"
#include "roboteam_tactics/Aggregator.h"

namespace rtt {

void Aggregator::putMsg(int robotID, const roboteam_msgs::SteeringGoal& goal) {
    auto client = getClient(robotID);

    // TODO: Maybe first aggregate the messages, and send them all out after the *entire* tree has traversed?
    // So instead of sending messages throughout the calculation of the trees, only send the messages after
    // the trees have been visited?
    printf("Sending new action! \n");
    client->sendGoal(goal);
}

actionlib::SimpleActionClient<roboteam_msgs::SteeringAction>* Aggregator::getClient(int robotID) {
    auto possibleClient = clients.find(robotID);
    if (possibleClient != clients.end()) {
        return possibleClient->second.get();
    } 

    clients[robotID] = std::unique_ptr<actionlib::SimpleActionClient<roboteam_msgs::SteeringAction>>(
            new actionlib::SimpleActionClient<roboteam_msgs::SteeringAction>("steering")
        );
    clients[robotID].get()->waitForServer();
    return clients[robotID].get();
}

actionlib::SimpleClientGoalState Aggregator::getState(int robotID) {
    auto client = getClient(robotID);

    return client->getState();
}

}
