#pragma once

#include <memory>
#include <vector>

#include "Blackboard.hpp"

namespace bt
{

class Node
{
public:
    enum class Status
    {
        Invalid,
        Success,
        Failure,
        Running,
    };

    virtual ~Node() {}

    virtual Status Update() = 0;
    virtual void Initialize() {}
    virtual void Terminate(Status s) {}

    Status Tick()
    {
        if (status != Status::Running) {
            Initialize();
        }

        status = Update();

        if (status != Status::Running) {
            Terminate(status);
        }

        return status;
    }

    bool IsSuccess() const { return status == Status::Success; }
    bool IsFailure() const { return status == Status::Failure; }
    bool IsRunning() const { return status == Status::Running; }
    bool IsTerminated() const { return IsSuccess() || IsFailure(); }
    void Reset() { status = Status::Invalid; }

    using Ptr = std::shared_ptr<Node>;

    bt::Blackboard::Ptr private_bb = std::make_shared<bt::Blackboard>();

protected:
    Status status = Status::Invalid;
};

using Nodes = std::vector<Node::Ptr>;

}
