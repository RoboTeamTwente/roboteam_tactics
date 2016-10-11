#!/usr/bin/env bash

treeSource="alltrees.cpp"
treeHeader="alltrees.h"

sourcePreamble="
#include \"roboteam_tactics/allskills.h\"
#include \"roboteam_tactics/allconditions.h\"
#include \"roboteam_tactics/alltactics.h\"
#include \"roboteam_tactics/bt.hpp\"
"

headerPreamble="
#pragma once
#include \"roboteam_tactics/bt.hpp\"
"

> $treeSource
> $treeHeader

printf "$sourcePreamble" >> $treeSource
printf "$headerPreamble" >> $treeHeader

printf "namespace rtt {\n" >> $treeSource
printf "namespace rtt {\n" >> $treeHeader

for filepath in ./json/*.json; do
    cat $filepath | ~/catkin_ws/devel/lib/roboteam_tactics/converter impl >> $treeSource
    printf "\n" >> $treeSource

	printf "\t" >> $treeHeader
	cat $filepath | ~/catkin_ws/devel/lib/roboteam_tactics/converter decl >> $treeHeader
done

printf "}" >> $treeSource
printf "}" >> $treeHeader

cp alltrees.h ../../include/roboteam_tactics/alltrees.h
rm alltrees.h
