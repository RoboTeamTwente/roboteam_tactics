#!/usr/bin/env bash

# TODO: Make sure it does built-in nodes as well. E.g. Repeat,
# and others that may come.

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

# Asserts that the script is being ran in tactics root
assert_tactics_root

dstFile="./all_custom_nodes.json"

> $dstFile

allNodes=()

# Appends a node item to allNodes array
# $1 Name + title of node
# $2 Category
function entry {
   node=$(printf "    {
        \"name\": \"$1\",
        \"category\": \"$2\",
        \"title\": \"$1\",
        \"description\": null,
        \"properties\": {}
    }")

    allNodes+=("$node")
}

for f in ./include/roboteam_tactics/skills/*.h; do
    name=$(basename "$f" .h)
	entry $name action
done

for f in ./include/roboteam_tactics/conditions/*.h; do
    name=$(basename "$f" .h)
	entry $name condition
done

for f in ./include/roboteam_tactics/tactics/*.h; do
    name=$(basename "$f" .h)
	entry $name action
done

result=$(join_by , "${allNodes[@]}")

printf "[\n" > $dstFile
printf "${result}" >> $dstFile
printf "\n]" >> $dstFile
