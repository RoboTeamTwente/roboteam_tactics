#include <memory>

#include "roboteam_tactics/bt/Node.hpp"
#include "roboteam_tactics/bt/Blackboard.hpp"
#include "roboteam_tactics/bt/Leaf.hpp"

namespace bt {

Leaf::Leaf() {}
Leaf::Leaf(Blackboard::Ptr blackboard) : blackboard(blackboard) {}

Leaf::~Leaf() {}

void Leaf::SetBlackboard(Blackboard::Ptr blackboard) {
    this->blackboard = blackboard; 
}

}
