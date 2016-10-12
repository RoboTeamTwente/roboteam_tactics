#!/usr/bin/env bash

# Usage:
#
# Command: ./generate.sh
# Should compile json converter if needed, and generate c++ source and header files.
#
# Command: ./generate.sh force
# Force compilation of the json converter, and generate c++ source and header files.

(
	# Go to src/treefen
	cd src/treegen
	# If the converter executable is not there OR there are more than 0 command line arguments
	if [ ! -f converter ] || [ "$#" -gt 0 ] ; then
		# Compile the converter
		g++ BTBuilder.cpp converter.cpp -std=c++11 -o converter -I ../../include
	fi
)

(
	# Run the convert script
	cd src/trees
	./convert.sh
)

(
	cd include/roboteam_tactics
	# To make sure that the pattern matches below return an empty string
	# in case of an empty directory
	shopt -s nullglob

	# For each include in various categories, generate an include file that includes them all
	printf "#pragma once\n" > allconditions.h
	for f in conditions/*.h; do
	    printf "#include \"$f\"\n" >> allconditions.h
	done

	printf "#pragma once\n" > allskills.h
	for f in skills/*.h; do
	    printf "#include \"$f\"\n" >> allskills.h
	done

	printf "#pragma once\n" > alltactics.h
	for f in tactics/*.h; do
	    printf "#include \"$f\"\n" >> alltactics.h
	done
)
