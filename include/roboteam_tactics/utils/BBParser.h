#pragma once

#include "roboteam_tactics/bt/Blackboard.hpp"
#include <string>
#include <vector>

namespace rtt {

void split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);
bt::Blackboard::Ptr parse_bb(const std::string& skill_name, const std::vector<std::string>& arguments);
bt::Blackboard::Ptr parse_bb(const std::string& name, const std::string& arg_fmt, ...);    
    
}