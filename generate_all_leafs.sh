#!/usr/bin/env bash

# Usage:
#
# Command: ./generate.sh
# Should compile json converter if needed, and generate c++ source and header files.
#
# DEPRECATED (but can be turned on if needed)
# Command: ./generate.sh force
# Force compilation of the json converter, and generate c++ source and header files.

# $1 - the include file
# $2 - the include pattern
# $3 - what to call the set variable
# $4 - the destination folder
function makeIncludeList {
    printf "#pragma once\n" > $4/$1
    for f in $2; do
        printf "#include \"roboteam_tactics/$f\"\n" >> $4/$1
    done

    setFile=$(basename "$1" .h)
    setFile+="_set.h"

    printf "#include <set>\n" > $4/$setFile
    printf "const std::set<std::string> $3 = {\n" >> $4/$setFile
    for f in $2; do
        hName=$(basename "$f" .h)
        printf "    \"$hName\",\n" >> $4/$setFile
    done
    printf "    \"element to accept the last comma\"\n" >> $4/$setFile
    printf "};\n" >> $4/$setFile
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
#include \"roboteam_tactics/allskills.h\"

namespace rtt {

/**
 * WARNING! ONLY USE FOR UNIT TESTS! REALLY!
 */
std::shared_ptr<Leaf> make_skill(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr bb) {
    if (false) {
        // Dummy condition
    } " >> generated/$factoryFile

    for f in skills/*.h; do
        skillName=$(basename "$f" .h) 
        printf "else if (name == \"$skillName\") {
        return std::make_shared<$skillName>(n, name, bb);
    } " >> generated/$factoryFile
    done

    # Postamble
    printf "else {
        std::cout << \"Error: Skill with name \" << name << \" not found.\\\\n\";
        exit(1)
    }

} // rtt
" >> generated/$factoryFile
)

#(
	## Go to src/treegen
	#cd src/treegen
	## If the converter executable is not there OR there are more than 0 command line arguments
	## if [ ! -f converter ] || [ "$#" -gt 0 ] ; then
		## Compile the converter
		## g++ BTBuilder.cpp converter.cpp -std=c++11 -o converter -I ../../include
        #make
	## fi
#)

#(
	## Run the convert script
	#cd src/trees
	#./convert.sh
#)
