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

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


std::string helpStr = R"V0G0N(
[Converter]

Description:
    Generates c++ functions from json files that can construct behavior trees.

Options:
    -namespace [namespace]
        Specifies the base namespace to put everything in. Is not included in the creation string.
    -f [folder]
        Indicates in which "folder" the tree needs to be put. Let's say the folder is "bobe/trees", and we're compile CoolTree.
        The functions will then end up in base_namespace::bobe::trees, ending up like base_namespace::bobe::trees::CoolTree(), and it's creation string (for nodefactory et al.)
        will be "bobe/trees/CoolTree".
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

} // anonymous namespace

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);

    if (cmdOptionExists(args, "-h") || cmdOptionExists(args, "--help")) {
        std::cout << helpStr;
        return 0;
    }

    bool fromStdIn = cmdOptionExists(args, "-stdin");
    bool toStdOut = cmdOptionExists(args, "-stdout");
    bool appendMode = cmdOptionExists(args, "-a");
    bool doImpl = cmdOptionExists(args, "-impl");
    bool doDecl = cmdOptionExists(args, "-decl");

    bool doNamespace = cmdOptionExists(args, "-namespace");
    bool doFolder = cmdOptionExists(args, "-f");

    std::string baseNamespace;
    if (doNamespace) {
        baseNamespace = getCmdOption(args, "-namespace");
    }

    std::string folder;
    if (doFolder) {
        folder = getCmdOption(args, "-f");
    }

    std::vector<std::string> namespaces = split(folder, '/');

    std::string input;

    if (fromStdIn) {
        input = readFileFromStdIn();
    } else {
        auto inputFile = getCmdOption(args, "-i");

        std::ifstream t(inputFile);
        std::stringstream buffer;
        buffer << t.rdbuf();
        input = buffer.str();
    }

    nlohmann::json info = nlohmann::json::parse(input);

    bool isTree = false;
    bool isProject = false;

    if (info.find("nodes") != info.end()) {
        isTree = true;
    } else if (info.find("data")  != info.end()) {
        isProject = true;
    } else {
        std::cerr << "JSON is not a B3 json structure; it doesn't have a nodes entry nor a data entry, and is therefore not a tree nor a project. Aborting.\n";
        return 1;
    }

    nlohmann::json allTrees = nlohmann::json::array();

    if (isTree) {
        allTrees.push_back(info);
    } else if (isProject) {
        allTrees = info["data"]["trees"];
    } else {
        std::cerr << "JSON is not a B3 json structure; it doesn't have a nodes entry nor a data entry, and is therefore not a tree nor a project. Aborting.\n";
        return 1;
    }

    if (doImpl) {
        // Function implementation 
        
        std::stringstream ss;
        for (auto& jsonTree : allTrees) {
            rtt::BTBuilder builder;
            ss << builder.build(jsonTree, baseNamespace, namespaces);
            ss << "\n";
        }

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

        if (doNamespace) {
            ss << "namespace " << baseNamespace << " {\n\n";
        }

        if (namespaces.size() > 0) {
            for (auto ns : namespaces) {
                ss << "namespace " << ns << " { ";
            }

            ss << "\n\n";
        }

        for (auto& jsonTree : allTrees) {
            ss << "bt::BehaviorTree make_" << jsonTree["title"].get<std::string>() << "(bt::Blackboard* blackboard = nullptr);\n";
        }

        ss << "\n";

        if (namespaces.size() > 0) {
            for (int i = namespaces.size() - 1; i > -1; --i) {
                ss << "} /* " << namespaces.at(i) << " */ ";
            }

            ss << "\n\n";
        }

        if (doNamespace) {
            ss << "} // " << baseNamespace << "\n";
        }

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
