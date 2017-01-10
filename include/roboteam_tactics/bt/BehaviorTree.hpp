#pragma once

#include "Node.hpp"
#include "Blackboard.hpp"

namespace bt
{

class BehaviorTree : public Node
{
public:
    BehaviorTree() : blackboard(std::make_shared<Blackboard>()) {}
    BehaviorTree(const Node::Ptr &rootNode) : BehaviorTree() { root = rootNode; }
    BehaviorTree(const Blackboard::Ptr &shared) : BehaviorTree() { sharedBlackboard = shared; }
    
    Status Update() override { return root->Tick(); }

    void Terminate(Status s) override {
        if (s == Node::Status::Running || s == Node::Status::Invalid) {
            root->Terminate(root->getStatus());
            s = Node::Status::Failure;
        }
    }
    
    void SetRoot(const Node::Ptr &node) { root = node; }
    Blackboard::Ptr GetBlackboard() const { return blackboard; }
    Blackboard::Ptr GetSharedBlackboard() const { return sharedBlackboard; }
    void SetSharedBlackboard(const Blackboard::Ptr &shared) { sharedBlackboard = shared; }
    
private:
    Node::Ptr root = nullptr;
    Blackboard::Ptr blackboard = nullptr;
    Blackboard::Ptr sharedBlackboard = nullptr;
};

}
