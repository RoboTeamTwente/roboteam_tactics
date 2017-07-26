#pragma once

#include <string>

#include "roboteam_tactics/bt.hpp"
#include "roboteam_utils/LastRef.h"

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
    
    // void AddChild(Node::Ptr child) final override;
    
    bt::Node::Status Update() override;

    void Terminate(Status s) override;
    
    std::string node_name() override;

    bt::Node::Ptr getCurrentChild();
    bt::Node::Ptr getPreviousChild();
    boost::optional<std::string> getCurrentStrategyTreeName() const;
    boost::optional<RefState> getCurrentRefState() const;
    bool hasStartedNewStrategy() const;

    void AddStrategy(RefState refState, Node::Ptr child);

    void printRefStateInfo() const;

private:
    bool validated;
    boost::optional<RefState> previousCmd;
    boost::optional<RefState> currentCmd;
    bool finishedOnce;
    bool needToInitialize;
    bool startedNewStrategy;
    unsigned lastKnownBotCount;
    // bool runningImplicitNormalStartRefCommand;
    
    std::map<RefState, Node::Ptr> refStateStrategies;
};
    
} // rtt
