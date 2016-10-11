#!/usr/bin/env bash

> allfuncs.cpp
> allfuncs.h

printf "namespace rtt {\n" >> allfuncs.cpp
printf "namespace rtt {\n" >> allfuncs.h

for filepath in ./json/*.json; do

    cat $filepath | ~/catkin_ws/devel/lib/roboteam_tactics/converter impl >> allfuncs.cpp
    printf "\n" >> allfuncs.cpp

	printf "\t" >> allfuncs.h
	cat $filepath | ~/catkin_ws/devel/lib/roboteam_tactics/converter decl >> allfuncs.h
done

printf "}" >> allfuncs.cpp
printf "}" >> allfuncs.h
