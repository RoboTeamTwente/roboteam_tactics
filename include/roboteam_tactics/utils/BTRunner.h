#include "roboteam_tactics/bt.hpp"

namespace rtt {
    
/**
 * \class BTRunner
 * \brief Runs a behavior tree, printing the current state every tick.
 * \deprecated This is no longer useful with the improved RQT view.
 */
class BTRunner {
    public:
    BTRunner(bt::BehaviorTree& tree, bool print_debug = false) : tree(tree), print_debug(print_debug) {}
    
    bt::Node::Status run_once();
    void run_until(bt::Node::Status status = bt::Node::Status::Success);
    
    template<typename Func>
    void run_until(Func f) {
        bt::Node::Status previousStatus = bt::Node::Status::Running;
        while (f(previousStatus)) { previousStatus = run_once(); }
    }
    
    private:
    bt::BehaviorTree tree;
    bool print_debug;
    void print();
}; 
    
}
