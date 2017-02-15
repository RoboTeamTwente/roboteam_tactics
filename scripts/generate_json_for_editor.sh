#!/usr/bin/env bash

# TODO: Make sure it does built-in nodes as well. E.g. Repeat,
# and others that may come.

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

# Asserts that the script is being ran in tactics root
assert_tactics_root

dstFile="./all_custom_nodes.json"

# TODO: Check for commented out lines.
paramRegex="\[\"(.*)\"\][ \t]*=[ \t]*BBArgumentType::"

> $dstFile

allNodes=()

# Appends a node item to allNodes array
# $1 Name + title of node
# $2 Category
# $3 properties
function entry {
    #printf "Registering: $1\n"
    node=$(printf "    {
        \"name\": \"$1\",
        \"category\": \"$2\",
        \"title\": \"$1\",
        \"description\": null,
        \"properties\": {$3}
    }")

    allNodes+=("$node")

}


# Gets the properties out of a header file.
# Stores the string into `properties`.
# $1 the file name
function getProperties {
    properties="\n"

    local first=true

    while read line
    do
        # Find all the properties.
        if [[ $line =~ $paramRegex ]]
        then
            if ! [ "${BASH_REMATCH[1]}" == "ROBOT_ID" ]
            then
                if ! $first
                then
                    properties+=",\n"
                fi
                properties+="\"${BASH_REMATCH[1]}\": \"\""
                local first=false
            fi
        fi
    done < $1
}

# Find all tactics recursively
for f in $(find ./include/roboteam_tactics/tactics -depth); do
    # Only look at paths that are files and that end in .h
    if [ -f $f ] && [[ $f == *.h ]] ; then
        # Figure out name & group/folder
        name=$(basename $f .h)
        group=$(dirname ${f#./include/roboteam_tactics/tactics/})

        # Decide whether or not the group is there
        entryName=""
        if [ "$group" != "." ]
        then
            entryName="$group/$name"
        else
            entryName="$name"
        fi

        # Get the properties and save the entry
        getProperties $f
        entry "$entryName" action "$properties"
    fi
done

for f in ./include/roboteam_tactics/conditions/*.h; do
    name=$(basename "$f" .h)
    getProperties $f
	entry $name condition "$properties"
done

for f in ./include/roboteam_tactics/tactics/*.h; do
    name=$(basename "$f" .h)
    getProperties $f
	entry $name action "$properties"
done

# Custom Composites/decorators here
entry ParallelTactic composite
entry ParallelSequence composite
entry Repeat decorator

result=$(join_by , "${allNodes[@]}")

printf "[\n" > $dstFile
printf "${result}" >> $dstFile
printf "\n]" >> $dstFile
