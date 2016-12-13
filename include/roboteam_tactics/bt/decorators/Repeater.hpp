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
    // TODO: constructor can be updated, ask Bob
    Repeater(int limit = 0) : limit(limit) {}

    void Initialize() override
    {
        counter = 0;
    }

    Status Update() override
    {
        while (1) {
            Node::append_status("[Repeater: executing child of type %s]", child->node_name().c_str());
            child->Tick();

            if (limit > 0 && ++counter == limit) {
                return Status::Success;
            }

            return Status::Running;
        }
    }
    std::string node_name() { return "Repeater"; }

protected:
    int limit;
    int counter = 0;
};

}
