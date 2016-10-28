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
    printf "#pragma once\n" > $4/$1
    for f in $2; do
        printf "#include \"roboteam_tactics/$f\"\n" >> $4/$1
    done

    baseFilename="$(basename "$1" .h)"
    make_aggregate_container "set" "$2" "${baseFilename}_set" "$4/${baseFilename}_set.h" "rtt"
    make_aggregate_container "vector" "$2" "${baseFilename}_list" "$4/${baseFilename}_list.h" "rtt"
}

(
	cd include/roboteam_tactics
	# To make sure that the pattern matches below return an empty string
	# in case of an empty directory
	shopt -s nullglob

	makeIncludeList allconditions.h "conditions/*.h" CONDITIONS generated

	makeIncludeList allskills.h "skills/*.h" SKILLS generated

	makeIncludeList alltactics.h "tactics/*.h" TACTICS generated

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

static std::shared_ptr<Leaf> make_skill(ros::NodeHandle n, std::string className, std::string name = \"\", bt::Blackboard::Ptr bb = nullptr) {
    if (false) {
        // Dummy condition
    } " >> generated/$factoryFile

    for f in skills/*.h; do
        skillName=$(basename "$f" .h) 
        printf "else if (className == \"$skillName\") {
        return std::make_shared<$skillName>(n, name, bb);
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
