#include "roboteam_tactics/bt/Composite.hpp"

namespace bt
{

Composite::~Composite() {}

void Composite::AddChild(Node::Ptr child) {
    children.push_back(child); 
}

bool Composite::HasNoChildren() const {
    return children.empty(); 
}

int Composite::GetIndex() const {
    return index; 
}

void Composite::Terminate(Status s) {
    for (auto child : children) {
        if (child->getStatus() == Status::Running) {
            child->Terminate(child->getStatus());
        }
    }

    if (s == Status::Running) {
        setStatus(Status::Failure);
    }
}
 
}
