#!/usr/bin/env bash

# Usage:
#
# Command: ./convert.sh
# Generates the files alltrees.h and alltrees.cpp, containing
# functions that construct all the behavior trees in the JSON folder.

# Go to the tree dir
cd src/trees

# Destination files
treeSource="alltrees.cpp"
treeHeader="alltrees.h"

# Preamble of the source file
sourcePreamble="
#include \"roboteam_tactics/generated/allskills.h\"
#include \"roboteam_tactics/generated/allconditions.h\"
#include \"roboteam_tactics/generated/alltactics.h\"
#include \"roboteam_tactics/bt.hpp\"
"

# Preamble of header file
headerPreamble="
#pragma once
#include \"roboteam_tactics/bt.hpp\"
#include \"roboteam_tactics/Aggregator.h\"
"

# Make sure destination files are empty
> $treeSource
> $treeHeader

# Append the preambles
printf "$sourcePreamble" >> $treeSource
printf "$headerPreamble" >> $treeHeader

# Append namespace specifiers
printf "namespace rtt {\n" >> $treeSource
printf "namespace rtt {\n" >> $treeHeader

# Generate declarations and implementations for the behavior trees
# And append them to the sourc and header files
for filepath in ./json/*.json; do
    # Generate implementations
    cat $filepath | rosrun roboteam_tactics converter impl >> $treeSource
    printf "\n" >> $treeSource

    # Generated header declarations
    printf "\t" >> $treeHeader
    cat $filepath | rosrun roboteam_tactics converter decl >> $treeHeader
done

# Closing brackets :D
printf "}" >> $treeSource
printf "}" >> $treeHeader

# Copy the header files to place catkin can find it and
# delete it here.
cp alltrees.h ../../include/roboteam_tactics/generated/alltrees.h
rm alltrees.h

###########################
## Make the tree factory ##
###########################

# Destination files
factorySource="alltrees_factory.cpp"
factoryHeader="alltrees_factory.h"

# Empty both files
> $factorySource
> $factoryHeader

# Source
printf "
#include <iostream>
#include <string>

#include \"ros/ros.h\"

#include \"roboteam_tactics/bt.hpp\"
#include \"roboteam_tactics/generated/alltrees.h\"

namespace rtt {

bt::BehaviorTree make_tree(std::string name, ros::NodeHandle n) {" >> $factorySource

printf "
    if (false) {
        // Bogus if clause
    }" >> $factorySource

for filepath in ./json/*.json; do
        name=$(basename $filepath .json)

        printf " else if (name == \"$name\") {
        return make_$name(n);
    } " >> $factorySource
done

printf "
   
    std::cout << \"Could not find tree with name \" << name << \". Aborting\\\\n\";
    exit(1);
}

}" >> $factorySource

# Header
printf "
#pragma once

#include <string>

#include \"ros/ros.h\"

#include \"roboteam_tactics/bt.hpp\"

namespace rtt {

bt::BehaviorTree make_tree(std::string name, ros::NodeHandle n);

}
" >> $factoryHeader

# Copy the header files to place catkin can find it and
# delete it here.
cp alltrees_factory.h ../../include/roboteam_tactics/generated/alltrees_factory.h
rm alltrees_factory.h
