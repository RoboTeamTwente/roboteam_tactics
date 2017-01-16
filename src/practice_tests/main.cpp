#include <memory>

#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_tactics/practice_tests/KeeperTest.h"

int main(int argc, char *argv[]) {
    using rtt::practice::KeeperTest;
    using rtt::practice::PracticeTest;

    std::unique_ptr<PracticeTest> keeperTest(new KeeperTest());
}
