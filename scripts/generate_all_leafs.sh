#!/usr/bin/env bash

# Usage:
#
# Execute with current working directory equal to roboteam_tactics root! 

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

assert_tactics_root

# $1 - the include file
# $2 - the include pattern
# $3 - what to call the set variable
# $4 - the destination folder
function makeIncludeList {
    mkdir -p $4

    printf "#pragma once\n" > $4/$1
    for f in $2; do
        printf "#include \"roboteam_tactics/$f\"\n" >> $4/$1
    done

    baseFilename="$(basename "$1" .h)"
    make_aggregate_container "set" "$2" "${baseFilename}_set" "$4/${baseFilename}_set.h" "rtt"
    make_aggregate_container "vector" "$2" "${baseFilename}_list" "$4/${baseFilename}_list.h" "rtt"
}

function make_typename_preamble {
    local dstFile=$1
    local namespace=$2
    >generated/$1
    printf "#pragma once

#include <string>
#include <exception>
#include \"allconditions.h\"
#include \"allskills.h\"
#include \"alltactics.h\"

namespace $namespace {

template<typename T>
std::string name_of_node(T* node) { throw std::logic_error(\"No generated name_of_node for a node type\"); }

" >> generated/$dstFile
}

function make_typename_postamble {
    local dstFile=$1
    printf "
} // namespace
" >> generated/$dstFile
}

function make_typename_funcs {
    local allFiles=$1
    local dstFile=$2

    for f in $allFiles; do
        local hName=$(basename "$f")
        hName="${hName%.*}"
        printf "template<>
std::string name_of_node<$hName>($hName* node) { return \"$hName\"; }

" >> generated/$dstFile
    done
}

(
	cd include/roboteam_tactics
	# To make sure that the pattern matches below return an empty string
	# in case of an empty directory
	shopt -s nullglob

    mkdir -p generated

	makeIncludeList allconditions.h "conditions/*.h" CONDITIONS generated

	makeIncludeList allskills.h "skills/*.h" SKILLS generated

	makeIncludeList alltactics.h "tactics/*.h" TACTICS generated

	make_typename_preamble typename_map.h rtt
    make_typename_funcs "conditions/*.h" typename_map.h
    make_typename_funcs "skills/*.h" typename_map.h
    make_typename_funcs "tactics/*.h" typename_map.h
    make_typename_postamble typename_map.h

    # 
    # Make skill factory
    # TODO: Factor this out in something generic...? Only if needed
    # for conditions & tactics too.
    #

    factoryFile="allskills_factory.h"
    # Clear the factory file
    > generated/$factoryFile

    # Preamble
    printf "#pragma once
#include <iostream>
#include <memory>
    
#include \"ros/ros.h\"

#include \"roboteam_tactics/bt.hpp\"
#include \"roboteam_tactics/generated/allskills.h\"

namespace rtt {

template <typename T=Leaf>
std::shared_ptr<T> make_skill(ros::NodeHandle n, std::string className, std::string name = \"\", bt::Blackboard::Ptr bb = nullptr) {
    if (false) {
        // Dummy condition
    } " >> generated/$factoryFile

    for f in skills/*.h; do
        skillName=$(basename "$f" .h) 
        printf "else if (className == \"$skillName\") {
        return std::dynamic_pointer_cast<T, $skillName>(std::make_shared<$skillName>(n, name, bb));
    } " >> generated/$factoryFile
    done

    # Postamble
    printf "else {
        std::cout << \"Error: Skill with name \" << className << \" not found.\\\\n\";
        exit(1);
    }
}

} // rtt
" >> generated/$factoryFile
)

mkdir -p generated
touch generated/generate_all_leafs.stamp
