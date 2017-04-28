#pragma once

#include <memory>
#include <vector>
#include <cstdio>
#include <cstdarg>

#include "Blackboard.hpp"

namespace bt {


class Node {
public:
    // When this is updated, updated the tostring method below too!
    enum class Status
    {
        Invalid,
        Success,
        Failure,
        Running,
    };

    virtual ~Node();

    virtual Status Update() = 0;
    virtual void Initialize();
    virtual void Terminate(Status s);

    virtual Status Tick();

    bool IsSuccess() const;
    bool IsFailure() const;
    bool IsRunning() const;
    bool IsTerminated() const;
    Status getStatus() const;
    void setStatus(Status s);

    using Ptr = std::shared_ptr<Node>;

    bt::Blackboard::Ptr private_bb = std::make_shared<bt::Blackboard>();

    virtual std::string node_name();
    static std::string status_desc;

protected:
    Status status = Status::Invalid;
    static void append_status(std::string fmt, ...);
};

using Nodes = std::vector<Node::Ptr>;

std::string statusToString(bt::Node::Status status);

} // bt
