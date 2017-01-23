#pragma once

#include <memory>
#include <vector>
#include <cstdio>
#include <cstdarg>

#include "Blackboard.hpp"


namespace bt
{


class Node
{
public:
    // When this is updated, updated the tostring method below too!
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
    virtual void Terminate(Status s) {
    }

    virtual Status Tick()
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
    
    virtual std::string node_name() { return "<ERROR>"; };
    static std::string status_desc;

protected:
    Status status = Status::Invalid;
    static void append_status(std::string fmt, ...) {
        char buf[1024];
        va_list varargs;
        va_start(varargs, fmt);
        vsnprintf(buf, 1024, fmt.c_str(), varargs);
        va_end(varargs);
        
        status_desc += std::string(buf);
    }
};

using Nodes = std::vector<Node::Ptr>;

static std::string statusToString(Node::Status status) {
    if (status == bt::Node::Status::Success) {
        return "Success";
    } else if (status == bt::Node::Status::Failure) {
        return "Failure";
    } else if (status == bt::Node::Status::Invalid) {
        return "Invalid";
    } else if (status == bt::Node::Status::Running) {
        return "Running";
    } else {
        std::cout << "Enum failure in Node::Status overload of to_string\n";
        return "ERROR ERROR!!!";
    }
}

}
