#pragma once

#include <tuple>
#include <boost/optional.hpp>
#include <string>

#include "roboteam_utils/json.hpp"

namespace rtt {

enum class CheckResult {
    Good,
    Bad,
    Error
} ;

std::tuple<CheckResult, boost::optional<std::string>> checkTree(nlohmann::json const & treeData, nlohmann::json const & customNodes);

} // rtt
