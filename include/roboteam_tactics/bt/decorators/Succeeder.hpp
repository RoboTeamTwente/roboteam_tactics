#pragma once

#include "../Decorator.hpp"

namespace bt
{

/*
    The Succeeder decorator returns success, regardless of what happens to the child.
*/
class Succeeder : public Decorator
{
public:
    Status Update() override
    {
        Node::append_status("[Succeeder: executing child of type %s]", child->node_name().c_str());
        child->Tick();
        return Status::Success;
    }
    std::string node_name() { return "Succeeder"; }
};

}