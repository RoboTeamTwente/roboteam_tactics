#pragma once

#include "../Decorator.hpp"

namespace bt
{

/*
    The Failer decorator returns failure, regardless of what happens to the child.
*/
class Failer : public Decorator
{
public:
    Status Update() override
    {
        Node::append_status("[Failer: executing child of type %s]", child->node_name().c_str());
        child->Tick();
        return Status::Failure;
    }
    std::string node_name() { return "Failer"; }
};

}