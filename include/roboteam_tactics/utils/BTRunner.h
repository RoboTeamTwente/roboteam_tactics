#include "roboteam_tactics/bt.hpp"

namespace rtt {
    
class BTRunner {
    public:
    BTRunner(bt::BehaviorTree& tree, bool print_debug = false) : tree(tree), print_debug(print_debug) {}
    
    bt::Node::Status run_once();
    void run_until(bt::Node::Status status = bt::Node::Status::Success);
    
    template<typename Func>
    void run_until(Func f) {
        while (f()) { run_once(); }
    }
    
    private:
    bt::BehaviorTree tree;
    bool print_debug;
    void print();
}; 
    
}