function enableColor {
    printf "\e[1;33m"
}

function disableColor {
    printf "\e[0m"
}

(
    cd ../../

    enableColor

    cat << EOM
-----------------------------------------------
-- Run catkin_make to discover all projects. --
-- This might fail; that's ok.               --
-----------------------------------------------
EOM

    disableColor

    catkin_make

    enableColor

    cat << EOM
-------------------------------------
-- Now we run catkin_make again to --
-- generate the proper code.       --
-------------------------------------
EOM

    disableColor

    catkin_make
)

enableColor

cat << EOM
------------------------------------
-- Now we touch CMakeLists.txt to --
-- trigger CMake to rebuild.      --
------------------------------------
EOM

disableColor

touch CMakeLists.txt

enableColor

cat << EOM
---------------------------------------
-- Now we clean to get rid of all    --
-- incorrectly compiled executables. --
---------------------------------------
EOM

disableColor

(
    cd ../../

    catkin_make clean

    enableColor
    
    cat << EOM
-------------------
-- Now we build! --
-------------------
EOM

    disableColor

    catkin_make
)

enableColor

cat << EOM
---------------------------------------------------
-- Your B3 project should be successfully added. --
-- Please ensure that the project and its tree   --
-- appear in the list below.                     --
---------------------------------------------------
EOM

disableColor

rosrun roboteam_tactics TestX show trees
