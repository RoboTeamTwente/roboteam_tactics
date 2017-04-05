#pragma once

#include <string>

#include "roboteam_tactics/bt.hpp"

namespace rtt {

/**
 * \class RefStateSwitch
 * \brief Top-level node which selects the correct strategy tree to use based on the current ref state.
 */
class RefStateSwitch : public bt::Composite {    
    
public:    
    RefStateSwitch();
    
    /**
     * \brief Checks whether this RefStateSwitch has all the children it should have.
     */
    bool isValid() const;
    
    /**
     * \brief Asserts that this RefStateSwitch should be in a valid state, and prints an error message if it is not.
     */
    void assertValid() const;
    
    void AddChild(Node::Ptr child) final override;
    
    bt::Node::Status Update() override;

    void Terminate(Status s) override;
    
    std::string node_name() override;

private:
    bool validated;
    int last;
};
    
} // rtt
