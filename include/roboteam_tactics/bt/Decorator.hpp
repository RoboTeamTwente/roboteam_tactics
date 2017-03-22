#pragma once

#include "Node.hpp"

namespace bt
{

class Decorator : public Node
{
public:
    virtual ~Decorator() {}

    void SetChild(Node::Ptr child) { this->child = child; }
    bool HasNoChild() const { return child == nullptr; }

    void Terminate(Status s) override {
        if (child->getStatus() == Status::Running) {
            child->Terminate(child->getStatus());
        }

        if (s == Status::Running) {
            setStatus(Status::Failure);
        }
    }
    
protected:
    Node::Ptr child = nullptr;
};

}
