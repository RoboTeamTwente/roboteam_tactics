#pragma once

#include "../Composite.hpp"

namespace bt
{

/*
    The MemSequence composite ticks each child node in order, and remembers what child it prevously tried to tick.
    If a child fails or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children succeeds, only then does the sequence succeed.
*/
class MemSequence : public Composite
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

            Node::append_status("[MemSequence: executing child of type %s]", child->node_name().c_str());
            auto status = child->Tick();

            // If the child fails, or keeps running, do the same.
            if (status != Status::Success) {
                return status;
            }

            index++;
        }

        return Status::Success;
    }
    
    std::string node_name() override { return "MemSequence"; }
    
    using Ptr = std::shared_ptr<MemSequence>;
};

static MemSequence::Ptr MakeMemSequence()
{
    return std::make_shared<MemSequence>();
}

}
