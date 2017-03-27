#!/usr/bin/env bash

# Usage:
#
# Command: ./generate_cpp_from_json_project.sh [path to b3 project file]
# Generates the files project_name_trees.cpp and project_name_trees.h and puts them in the trees directory & generated directory for headers

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

# Asserts that the script is being ran in tactics root
assert_tactics_root

projectPath=$1
projectName=$(basename "$projectPath" ".b3")

## Destination files
projectSource="src/trees/${projectName}_trees.cpp"
projectHeader="include/roboteam_tactics/generated/${projectName}_trees.h"

## Construct folders in case of fresh build
mkdir -p $(dirname $projectSource)
mkdir -p $(dirname $projectHeader)

# Preamble of source file
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
> $projectSource
> $projectHeader

# Append the preambles
printf "$sourcePreamble" >> $projectSource
printf "$headerPreamble" >> $projectHeader

# Generate declarations and implementations for the behavior trees
# And append them to the sourc and header files
# The { exit 1; } makes sure the script exits with one if it fails
rosrun roboteam_tactics converter -impl -i "$projectPath" -o "$projectSource" -namespace rtt -f $projectName -a || { exit 1; }

rosrun roboteam_tactics converter -decl -i "$projectPath" -o "$projectHeader" -namespace rtt -f $projectName -a || { exit 1; }

# mkdir -p generated
# touch generated/generate_cpp_from_json_project_${projectName}.stamp

