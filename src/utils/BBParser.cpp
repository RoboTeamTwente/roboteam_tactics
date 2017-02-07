#include "roboteam_tactics/utils/BBParser.h"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/utils/BBParser.h"

namespace rtt {

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

bt::Blackboard::Ptr parse_bb(const std::string& skill_name, const std::vector<std::string>& arguments) {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    std::string prefix = skill_name == "" ? "" : skill_name + "_";
    for (size_t i = 0; i < arguments.size(); i++) {
        std::vector<std::string> typeSplit;
        
        auto arg = arguments.at(i);
        
        auto nameSplit = split(arg, '=');
        auto name = prefix + nameSplit.at(0);
        auto rest = nameSplit.at(1);
        
        // Aggregate all the splitted = into one string
        // This happens if you try to set a value that contains multiple equals
        // (Then you only want to split on the first)
        for (size_t i = 2; i < nameSplit.size(); i++) {
            rest += nameSplit.at(i);
        }

        std::string argType;
        if (name.find(":") != std::string::npos) {
            // Name contains type - lets take it out
            auto typeSplit = split(name, ':');
            argType = typeSplit.at(0);
            name = prefix + typeSplit.at(1);
        } else {
            // Derive type
            if (rest == "true") {
                argType = "bool";
            } else if (rest == "false") {
                argType = "bool";
            } else if (rest.find(".") != std::string::npos) {
                argType = "double";
            } else if (is_digits(rest)) {
                argType = "int";
            } else {
                argType = "string";
            }
        }
        
        
        // Uncomment to see the arguments
        // std::cout << "\n[Arg]\n";
        // std::cout << "Type: " << argType << "\n";
        // std::cout << "Name: " << name << "\n";
        // std::cout << "Value: " << rest << "\n";

        if (argType == "string") {
            bb->SetString(name, rest);
        } else if (argType == "int") {
            bb->SetInt(name, std::stoi(rest));
        } else if (argType == "double") {
            bb->SetDouble(name, std::stod(rest));
        } else if (argType == "bool") {
            bb->SetBool(name, rest == "true");
        } else {
            std::cout << "Unknown arg type: " << argType << "\n";
        }
    }
    return bb;
}
  
bt::Blackboard::Ptr parse_bb(const std::string& name, const std::string& arg_fmt, ...) {
    char buf[1024];
    va_list varargs;
    va_start(varargs, arg_fmt);
    vsnprintf(buf, 1024, arg_fmt.c_str(), varargs);
    va_end(varargs);
    
    std::string args(buf);
    return parse_bb(name, split(args, ' '));
}

}