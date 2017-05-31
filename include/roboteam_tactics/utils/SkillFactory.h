#pragma once

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
    
// build_skill(name, arg_fmt, args...)

/**
 * \brief Build a skill from a name and a TestX-formatted blackboard string
 * \tparam T The type of the skill to create
 * \param type_name The name of the skill class
 * \param skill_name The name of the node; as listed in the behavior tree
 * \param arg_fmt A format string which will constitute the blackboard
 * \param <varargs> All arguments needed to make arg_fmt a valid blackboard string.
 * \return A pointer to the finished skill
 * \throws invalid_argument If either blackboard validation is enabled for T, and it failed, or if
 * no entry for this skill type exists in the skills Repo.
 */
template <typename T>
std::shared_ptr<T> build_skill(const std::string& type_name, const std::string& skill_name, const std::string& arg_fmt, ...) {
    char buf[1024];
    va_list varargs;
    va_start(varargs, arg_fmt);
    vsnprintf(buf, 1024, arg_fmt.c_str(), varargs);
    va_end(varargs);
    
    std::string args(buf);
    bt::Blackboard::Ptr bb = parse_bb(skill_name, split(args, ' '));
    if (!Leaf::validate_blackboard<T>(bb, skill_name)) {
        throw std::invalid_argument("Blackboard verification failed - Check ROS logs."); 
    }

    namespace f = rtt::factories;

    auto& repo = f::getRepo<f::Factory<Skill>>();
    if (repo.find(type_name) == repo.end()) {
        throw std::invalid_argument("Could not construct skill of type \"" + type_name + "\""); 
    }

    auto skillFactory = repo.at(type_name);

    return std::dynamic_pointer_cast<T>(skillFactory(skill_name, bb));
}
    
}
