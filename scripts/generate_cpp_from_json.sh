#!/usr/bin/env bash

# Usage:
#
# Command: ./generate_cpp_from_json.sh [--no-mem-principle]
# Generates the files alltrees.h and alltrees.cpp, containing
# functions that construct all the behavior trees in the JSON folder.

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

# Asserts that the script is being ran in tactics root
assert_tactics_root

# Destination files
treeSource="src/trees/alltrees.cpp"
treeHeader="include/roboteam_tactics/generated/alltrees.h"

# Construct folders in case of fresh build
mkdir -p $(dirname $treeSource)
mkdir -p $(dirname $treeHeader)

sourcePreamble="\
#include \"roboteam_tactics/bt.hpp\"
#include \"roboteam_tactics/utils/ParallelTactic.hpp\"
#include \"roboteam_tactics/utils/utils.h\"
#include \"roboteam_tactics/treegen/LeafRegister.h\"

"

# Preamble of header file
headerPreamble="\
#pragma once
#include \"roboteam_tactics/bt.hpp\"

"

# Make sure destination files are empty
> $treeSource
> $treeHeader

# Append the preambles
printf "$sourcePreamble" >> $treeSource
printf "$headerPreamble" >> $treeHeader

# Generate declarations and implementations for the behavior trees
# And append them to the sourc and header files

# for filepath in src/trees/json/*.json; do
#     # Generate implementations
#     echo "[generate_cpp_from_json.sh] Running converter for ${filepath}"
#     rosrun roboteam_tactics converter -impl -a -i "$filepath" -o "$treeSource" -namespace rtt $1 || { exit 1; }
#     printf "\n" >> $treeSource

#     # Generated header declarations
#     rosrun roboteam_tactics converter -decl -a -i "$filepath" -o "$treeHeader" -namespace rtt $1 || { exit 1; }
#     printf "\n" >> $treeHeader
# done

# mkdir -p generated
# touch generated/generate_cpp_from_json.stamp
