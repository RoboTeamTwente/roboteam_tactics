#pragma once

#include "../Decorator.hpp"

namespace bt
{

/*
    The Repeater decorator repeats infinitely or to a limit until the child returns success.
*/
class Repeater : public Decorator
{
public:
    Repeater(int limit = 0) : limit(limit) {}

    void Initialize() override
    {
        counter = 0;
    }

    Status Update() override
    {
        while (1) {
            Node::append_status("[Repeater: executing child of type %s]", child->node_name().c_str());
            auto s = child->Tick();

            if (s == Status::Running) {
                return Status::Running;
            }

            if (s == Status::Failure) {
                return Status::Failure;
            }

            if (limit > 0 && ++counter == limit) {
                return Status::Success;
            }

            child->Reset();
        }
    }
    std::string node_name() { return "Repeater"; }

protected:
    int limit;
    int counter = 0;
};

}
