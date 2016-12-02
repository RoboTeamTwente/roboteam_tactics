#include <iostream>
#include <string>

#include "roboteam_tactics/treegen/BTBuilder.h"
#include "roboteam_tactics/treegen/json.hpp"

#include "roboteam_tactics/skills/GetBall.h"

int main(int argc, char** argv) {
    std::string line;
    std::string input;
    while(std::getline(std::cin, line)) {
        input += line + "\n";
    }

    std::vector<std::string> arguments(argv + 1, argv + argc);

    auto info = nlohmann::json::parse(input);

    if (arguments.size() == 1 && arguments[0] == "impl") {
        // Function implementation
        rtt::BTBuilder builder;
        std::cout << builder.build(info);
    } else if (arguments.size() == 1 && arguments[0] == "decl") {
        // Function definition
        std::cout << "bt::BehaviorTree make_" << info["title"].get<std::string>() << "(bt::Blackboard* blackboard = nullptr);\n";
    }

}
