#pragma once

#include "../Decorator.hpp"

namespace bt
{

/*
    The UntilFail decorator repeats until the child returns fail and then returns success.
*/
class UntilFail : public Decorator
{
public:
    Status Update() override
    {
        while (1) {
            auto status = child->Tick();

            if (status == Status::Failure) {
                return Status::Success;
            } else if (status == Status::Running) {
                return Status::Running;
            }
        }
    }
};

}