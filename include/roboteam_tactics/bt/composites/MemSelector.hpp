#pragma once

#include "../Composite.hpp"

namespace bt
{

/*
    The Selector composite ticks each child node in order, and remembers what child it prevously tried to tick.
    If a child succeeds or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children fails, only then does the selector fail.
*/
class MemSelector : public Composite
{
public:
    size_t index;

    void Initialize() override {
        index = 0;
    }

    Status Update() override {
        if (HasNoChildren()) {
            return Status::Success;
        }

        // Keep going until a child behavior says it's running.
        while (index < children.size()) {
            auto &child = children.at(index);
            Node::append_status("[MemSelector: executing child of type %s]", child->node_name().c_str());
            auto status = child->Tick();

            // If the child succeeds, or keeps running, do the same.
            if (status != Status::Failure) {
                return status;
            }

            index++;
        }

        return Status::Failure;
    }
    
    using Ptr = std::shared_ptr<MemSelector>;
    
    std::string node_name() override { return "MemSelector"; }
};

static MemSelector::Ptr MakeMemSelector()
{
    return std::make_shared<MemSelector>();
}

}
