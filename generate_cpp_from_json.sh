#!/usr/bin/env bash

# Usage:
#
# Command: ./convert.sh
# Generates the files alltrees.h and alltrees.cpp, containing
# functions that construct all the behavior trees in the JSON folder.

echo "Generating c++ from json using allconditions.h et al."

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
    cat $filepath | rosrun roboteam_tactics converter impl >> $treeSource
    printf "\n" >> $treeSource

    printf "\t" >> $treeHeader
    cat $filepath | rosrun roboteam_tactics converter decl >> $treeHeader

    # Before cmake integration
    #cat $filepath | ../treegen/converter impl >> $treeSource
    #printf "\n" >> $treeSource

    #printf "\t" >> $treeHeader
    #cat $filepath | ../treegen/converter decl >> $treeHeader
done

# Closing brackets :D
printf "}" >> $treeSource
printf "}" >> $treeHeader

# Copy the header files to place catkin can find it and
# delete it here.
cp alltrees.h ../../include/roboteam_tactics/generated/alltrees.h
rm alltrees.h
