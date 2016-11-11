#pragma once

#include "../Decorator.hpp"

namespace bt
{

/*
    The UntilSuccess decorator repeats until the child returns success and then returns success.
*/
class UntilSuccess : public Decorator
{
public:
    Status Update() override
    {
        while (1) {
            Node::append_status("[UntilSuccess: executing child of type %s]", child->node_name().c_str());
            auto status = child->Tick();

            if (status == Status::Success) {
                return Status::Success;
            } else if (status == Status::Running) {
                return Status::Running;
            }
        }
    }
    std::string node_name() { return "UntilSuccess"; }
};

}