#!/usr/bin/env bash

# Usage:
#
# Command: ./convert.sh
# Generates the files alltrees.h and alltrees.cpp, containing
# functions that construct all the behavior trees in the JSON folder.

# Get the shared code
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/shared.sh

# Asserts that the script is being ran in tactics root
assert_tactics_root

(
    # Go to the tree dir
    cd src/trees

    # Destination files
    treeSource="alltrees.cpp"
    treeHeader="alltrees.h"

    # Preamble of the source file
    #include \"roboteam_tactics/generated/allskills.h\"
    #include \"roboteam_tactics/generated/allconditions.h\"
    #include \"roboteam_tactics/generated/alltactics.h\"
    sourcePreamble="
    #include \"roboteam_tactics/bt.hpp\"
    #include \"roboteam_tactics/utils/ParallelTactic.hpp\"
    #include \"roboteam_tactics/utils/utils.h\"
    #include \"roboteam_tactics/treegen/LeafRegister.h\"

"

    # Preamble of header file
    headerPreamble="
    #pragma once
    #include \"roboteam_tactics/bt.hpp\"
    "

    # Make sure destination files are empty
    > $treeSource
    > $treeHeader

    # Append the preambles
    printf "$sourcePreamble" >> $treeSource
    printf "$headerPreamble" >> $treeHeader

    # Append namespace specifiers
    #printf "namespace rtt {\n" >> $treeSource
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
    #printf "}" >> $treeSource
    printf "}" >> $treeHeader

    # Copy the header files to place catkin can find it and
    # delete it here.
    mkdir -p ../../include/roboteam_tactics/generated
    cp alltrees.h ../../include/roboteam_tactics/generated/alltrees.h
    rm alltrees.h

    ###########################
    ## Make the tree factory ##
    ###########################

    ## Destination files
    #factorySource="alltrees_factory.cpp"
    #factoryHeader="alltrees_factory.h"

    ## Empty both files
    #> $factorySource
    #> $factoryHeader

    ## Source
    #printf "
    ##include <iostream>
    ##include <string>

    ##include \"ros/ros.h\"

    ##include \"roboteam_tactics/bt.hpp\"
    ##include \"roboteam_tactics/generated/alltrees.h\"

    #namespace rtt {

    #bt::BehaviorTree make_tree(std::string name, bt::Blackboard* bb) {" >> $factorySource

    #printf "
        #if (false) {
            #// Bogus if clause
        #}" >> $factorySource

    #for filepath in ./json/*.json; do
            #name=$(basename $filepath .json)

            #printf " else if (name == \"$name\") {
            #return make_$name(bb);
        #} " >> $factorySource
    #done

    #printf "
       
        #std::cout << \"Could not find tree with name \" << name << \". Aborting\\\\n\";
        #exit(1);
    #}

    #}" >> $factorySource

    ## Header
    #printf "
    ##pragma once

    ##include <string>

    ##include \"ros/ros.h\"

    ##include \"roboteam_tactics/bt.hpp\"

    #namespace rtt {

    #bt::BehaviorTree make_tree(std::string name, bt::Blackboard* bb = nullptr);

    #}
    #" >> $factoryHeader

    ## Make set & lists for all json trees
    #make_aggregate_container "set" "./json/*.json" "alltrees_set" "alltrees_set.h" "rtt"

    #make_aggregate_container "vector" "./json/*.json" "alltrees_list" "alltrees_list.h" "rtt"

    ## Copy the header files to place catkin can find it and
    ## delete it here.
    ## $1 - the header file
    #function saveHeader {
        #cp $1 ../../include/roboteam_tactics/generated/$1
        #rm $1
    #}

    #saveHeader alltrees_factory.h
    #saveHeader alltrees_list.h
    #saveHeader alltrees_set.h

)

mkdir -p generated
touch generated/generate_cpp_from_json.stamp
