#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "roboteam_tactics/treegen/BTBuilder.h"
#include "roboteam_tactics/treegen/json.hpp"

namespace {

std::string getCmdOption(const std::vector<std::string>& args, const std::string & option) {
    auto it = std::find(args.begin(), args.end(), option);
    if (it != args.end() && (it + 1) != args.end()) {
        return *(it + 1);
    }

    std::cout << "[Converter] Error: command option \"" << option << "\" not supplied.\n";
    std::exit(EXIT_FAILURE);
}

bool cmdOptionExists(const std::vector<std::string>& args, const std::string& option) {
    return std::find(args.begin(), args.end(), option) != args.end();
}

std::string readFileFromStdIn() {
    std::string line;
    std::string input;
    while(std::getline(std::cin, line)) {
        input += line + "\n";
    }
    
    return input;
}

} // anonymous namespace

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);

    if (cmdOptionExists(args, "-h") || cmdOptionExists(args, "--help")) {
        std::cout << R"V0G0N(
[Converter]

Description:
    Generates c++ functions from json files that can construct behavior trees.

Options:
    -i [input file]
        Uses [input file] to read json
    -o [output file]
        Puts the generated c++ code in [output file]. Truncates the target file unless -a is used.
    -stdin
        Makes converter read from stdin. Overrides -i.
    -stdout
        Makes converter print its results on stdout. Overrides -o.
    -a
        If used in conjuction with -o, appends to the file instead of truncating it.
    -impl
        Indicates that the implementation of the c++ code should be generated
    -decl
        Indicates that the declaration of the implementation (forward declaration of the function) should be generated.

)V0G0N";
        
        return 0;
    }

    bool fromStdIn = cmdOptionExists(args, "-stdin");
    bool toStdOut = cmdOptionExists(args, "-stdout");
    bool appendMode = cmdOptionExists(args, "-a");
    bool doImpl = cmdOptionExists(args, "-impl");
    bool doDecl = cmdOptionExists(args, "-decl");

    std::string input;
    nlohmann::json info;

    if (fromStdIn) {
        input = readFileFromStdIn();
    } else {
        auto inputFile = getCmdOption(args, "-i");

        std::ifstream t(inputFile);
        std::stringstream buffer;
        buffer << t.rdbuf();
        input = buffer.str();
    }

    info = nlohmann::json::parse(input);

    if (doImpl) {
        // Function implementation 
        
        rtt::BTBuilder builder;

        std::stringstream ss;
        ss << builder.build(info);

        if (toStdOut) {
            std::cout << ss.str();
        } else {
            std::ios_base::openmode mode = std::ios_base::out;

            if (appendMode) {
                mode |= std::ios_base::app;
            } else {
                mode |= std::ios_base::trunc;
            }

            auto outputFile = getCmdOption(args, "-o");
            std::ofstream ofs(outputFile.c_str(), mode);
            ofs << ss.str();
        }
    } else if (doDecl) {
        // Function definition
        
        std::stringstream ss;
        ss << "bt::BehaviorTree make_" << info["title"].get<std::string>() << "(bt::Blackboard* blackboard = nullptr);\n";

        if (toStdOut) {
            std::cout << ss.str();
        } else {
            std::ios_base::openmode mode = std::ios_base::out;

            if (appendMode) {
                mode |= std::ios_base::app;
            } else {
                mode |= std::ios_base::trunc;
            }

            auto outputFile = getCmdOption(args, "-o");
            std::ofstream ofs(outputFile.c_str(), mode);
            ofs << ss.str();
        }
    }
}
