#pragma once

#include "../Composite.hpp"

namespace bt
{

/*
    The Selector composite ticks each child node in order.
    If a child succeeds or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children fails, only then does the selector fail.
*/
class Selector : public Composite
{
public:
    Status Update() override
    {
        // Keep going until a child behavior says it's running.
        for (auto& child : children) {
            Node::append_status("[Selector: executing child of type %s]", child->node_name().c_str());
            auto status = child->Tick();

            // If the child succeeds, or keeps running, do the same.
            if (status != Status::Failure) {
                return status;
            }
        }

        return Status::Failure;
    }
    
    std::string node_name() override { return "Selector"; }
    
    using Ptr = std::shared_ptr<Selector>;
};

static Selector::Ptr MakeSelector()
{
    return std::make_shared<Selector>();
}

}
