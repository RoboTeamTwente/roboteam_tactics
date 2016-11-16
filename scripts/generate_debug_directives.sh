#!/usr/bin/env bash

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

# Asserts that the script is being ran in tactics root
assert_tactics_root

NAME_FILE="include/roboteam_tactics/debug_names.txt"
OUTPUT_FILE="include/roboteam_tactics/generated/debug.h"
>$OUTPUT_FILE

printf "#pragma once

" >> $OUTPUT_FILE

while IFS= read -r name
do
    printf "#ifdef DEBUG_$name
#define DEBUG_INFO_$name(msg,...) ROS_INFO(msg, ##__VA_ARGS__);
#define DEBUG_WARN_$name(msg,...) ROS_WARN(msg, ##__VA_ARGS__);
#define DEBUG_ERROR_$name(msg,...) ROS_ERROR(msg, ##__VA_ARGS__);
#else
#define DEBUG_INFO_$name(msg,...)
#define DEBUG_WARN_$name(msg,...)
#define DEBUG_ERROR_$name(msg,...)
#endif

" >> $OUTPUT_FILE
done < $NAME_FILE

mkdir -p generated
touch generated/generate_debug_directives.stamp
