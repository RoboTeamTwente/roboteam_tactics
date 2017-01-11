#pragma once

#include "../Composite.hpp"

namespace bt
{

/*
    The Sequence composite ticks each child node in order.
    If a child fails or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children succeeds, only then does the sequence succeed.
*/
class Sequence : public Composite {
public:
    Status Update() override
    {
        if (HasNoChildren()) {
            return Status::Success;
        }
        
        // Keep going until a child behavior says it's running.
        for (auto &child : children) {
            Node::append_status("[Sequence: executing child of type %s]", child->node_name().c_str());
            auto status = child->Tick();
            
            // If the child fails, or keeps running, do the same.
            if (status != Status::Success) {
                return status;
            }
        }

        return Status::Success;
    }
    
    std::string node_name() { return "Sequence"; }
    
    using Ptr = std::shared_ptr<Sequence>;
};

static Sequence::Ptr MakeSequence()
{
    return std::make_shared<Sequence>();
}

}
