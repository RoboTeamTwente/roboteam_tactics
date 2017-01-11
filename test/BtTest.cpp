#include <gtest/gtest.h>

#include "roboteam_tactics/bt.hpp"

namespace {

    std::vector<std::string> traces;

    class Tracer : public bt::Leaf {
    public:
        std::string id;

        Tracer(std::string id) : id{id} {}

        void Initialize() override {
            traces.push_back("Initialize: " + id);

            Initialize_();
        }

        Status Update() override {
            traces.push_back("Update: " + id);

            return Update_();
        }

        void Terminate(Status s) override {
            traces.push_back("Terminate: " + id);

            Terminate_(s);
        }

        // Spoofed functions
        
        virtual void Initialize_() {

        }

        virtual Status Update_() {
            return Status::Success;
        }

        virtual void Terminate_(Status s) {
        
        }
    } ;

    class Once : public Tracer {
    public:
        Once(std::string id) : Tracer("Once-" + id) {}
    }; 

    class Runner : public Tracer {
    public:
        Runner(std::string id) : Tracer("Runner-" + id) {}

        Status Update_() override {
            return Status::Running;
        }

        void Terminate_(Status s) override {
            if (s == Status::Running || s == Status::Invalid) {
                setStatus(Status::Failure);
            }
        }
    } ;

    class Counter : public Tracer {
    public:
        int max;
        int runningCount;

        Counter(std::string id, int max)
            : Tracer("Counter-" + id)
            , max{max}
            , runningCount{0} {}

        void Initialize_() override {
            runningCount = 0;
        }

        Status Update_() override {
            runningCount++;

            if (runningCount == max) {
                return Status::Success;
            }
            
            return Status::Running;
        }

        void Terminate_(Status s) override {
            if (s == Status::Running || s == Status::Invalid) {
                setStatus(Status::Failure);
            }
        }

    } ;

    void printTrace() {
        std::cout << "Trace: \n";
        for (auto trace : traces) {
            std::cout << "\t" << trace << "\n";
        }
    }

} // anonymous namespace

// This is an abomination
std::string bt::Node::status_desc;

/////////////////////////////////
// Behavior Tree with one leaf //
/////////////////////////////////

TEST(BehaviorTreeTest, BehaviorTreeWithOneLeaf) {
    {
        bt::Leaf::Ptr once = std::make_shared<Once>("A");
        bt::BehaviorTree bt(once);
        
        traces.clear();
        bt.Tick();

        std::vector<std::string> expectedTrace = {
            "Initialize: Once-A",
            "Update: Once-A",
            "Terminate: Once-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }

    {
        bt::Leaf::Ptr runner = std::make_shared<Runner>("A");
        bt::BehaviorTree bt(runner);
        
        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();
        bt.Terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
            "Initialize: Runner-A",
            "Update: Runner-A",
            "Update: Runner-A",
            "Update: Runner-A",
            "Terminate: Runner-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }

    {
        bt::Leaf::Ptr counter = std::make_shared<Counter>("A", 3);
        bt::BehaviorTree bt(counter);
        
        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();

        std::vector<std::string> expectedTrace = {
            "Initialize: Counter-A",
            "Update: Counter-A",
            "Update: Counter-A",
            "Update: Counter-A",
            "Terminate: Counter-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Success);
    }

    {
        bt::Leaf::Ptr counter = std::make_shared<Counter>("A", 5);
        bt::BehaviorTree bt(counter);
        
        traces.clear();
        bt.Tick();
        bt.Tick();
        bt.Tick();
        bt.Terminate(bt.getStatus());

        std::vector<std::string> expectedTrace = {
            "Initialize: Counter-A",
            "Update: Counter-A",
            "Update: Counter-A",
            "Update: Counter-A",
            "Terminate: Counter-A"
        };

        ASSERT_EQ(expectedTrace, traces);
        ASSERT_EQ(bt.getStatus(), bt::Node::Status::Failure);
    }
}

/////////////////////////////////////////////
// Behavior Tree with counters & sequences //
/////////////////////////////////////////////

TEST(BehaviorTreeTest, BehaviorTreeWithSequencesAndCounters) {
    
}
