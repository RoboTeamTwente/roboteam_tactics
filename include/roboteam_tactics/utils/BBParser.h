#pragma once

#include "roboteam_tactics/bt/Blackboard.hpp"
#include <string>
#include <vector>

namespace rtt {

/**
 * \brief Splits a string by a certain delimiter.
 * The resulting fragments will not include the delimiters
 * \param s The string to split
 * \param delim The delimiter
 * \param elems The vector in which to storethe results
 */
void split(const std::string &s, char delim, std::vector<std::string> &elems);

/**
 * \brief Splits a string by a certain delimiter.
 * The resulting fragments will not include the delimiters
 * \param s The string to split
 * \param delim The delimiter
 * \return The vector in which to storethe results
 */
std::vector<std::string> split(const std::string &s, char delim);

/**
 * \brief Parses a string into a blackboard.
 * 
 * bb ::= <param>+
 * param ::= (<type> ":")? [\w_][\w\d_]* "=" <value>
 * type ::= "int" | "double" | "bool" | "string"
 * value ::= [0-9]+
 *         | [0-9]+ "." [0-9]+
 *         | "true"
 *         | "false"
 *         | "\"" .*? "\""
 * 
 * \param skill_name The name of the skill for which the blackboard is being built. This is used as a prefix
 * for the parameters.
 * \param arguments The arguments in the above format, split into separate strings.
 * \return A pointer to the finished blackboard.
 */
bt::Blackboard::Ptr parse_bb(const std::string& skill_name, const std::vector<std::string>& arguments);

/**
 * \brief Parses a string into a blackboard.
 * 
 * bb ::= <param>+
 * param ::= (<type> ":")? [\w_][\w\d_]* "=" <value>
 * type ::= "int" | "double" | "bool" | "string"
 * value ::= [0-9]+
 *         | [0-9]+ "." [0-9]+
 *         | "true"
 *         | "false"
 *         | "\"" .*? "\""
 * 
 * \param skill_name The name of the skill for which the blackboard is being built. This is used as a prefix
 * for the parameters.
 * \param arg_fmt A printf-like format string to place values in prior to parsing.
 * \param <varargs> The arguments matching the format string.
 * \return A pointer to the finished blackboard.
 */
bt::Blackboard::Ptr parse_bb(const std::string& name, const std::string& arg_fmt, ...);    
    
}