# Exits the script if the current working directory is not roboteam tactics
function assert_tactics_root {
    if [[ "$PWD" != */src/roboteam_tactics ]]; then
        printf "Invalid working directory; should be roboteam tactics root. Aborting.\n"
        exit 1
    fi
}

# Joins an array of strings in argument $2 with the sep character or string
# in arg $1. No clue how it works.
function join_by {
    local d=$1; shift; echo -n "$1"; shift; printf "%s" "${@/#/$d}";
}

# Makes a header file containing a constant container with as elements
# the filenames (without extensions) in the directory passed to the function.
# $1 - container type (set/vector/...)
# $2 - aggregator string (./.json/*.json)
# $3 - variable name
# $4 - destination file
# $5 - namespace (optional)
function make_aggregate_container {
    # Put args in clear variables
    local containerType=$1
    local allFiles=$2
    local varName=$3
    local dstFile=$4
    local namespace=$5

    # Header preamble
    printf "#pragma once

#include <${containerType}>
#include <string>

" > $dstFile

    # If fifth argument is passed then add an enclosing namespace
    if [ $# -eq 5 ]; then
        printf "namespace $namespace {

" >> $dstFile
    fi 

    printf "const std::${containerType}<std::string> ${varName} = {\n" >> $dstFile

    local allEntries=()

    # For each file in path allFiles...
    for f in $allFiles; do
        # Get the filename from the path
        local hName=$(basename "$f")
        # Get rid of the extension
        hName="${hName%.*}"
        # Add is as a string in the list allEntries
        allEntries+=("    \"$hName\"")
    done

    # Join all the elements with commas & newlines
    allEntries=$(join_by ",\n" "${allEntries[@]}")

    # Write it all to file
    printf "$allEntries" >> $dstFile

    # Close the literal
    printf "\n};\n" >> $dstFile

    # If five arguments are passed, close the enclosing namespace
    if [ $# -eq 5 ]; then
        printf "
};" >> $dstFile
    fi 
}
