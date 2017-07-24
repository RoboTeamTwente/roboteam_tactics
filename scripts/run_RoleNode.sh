#!/bin/bash

rosrun roboteam_tactics RoleNode "$@"
while [ $? -eq 42 ]; do
  rosrun roboteam_tactics RoleNode "$@"
done
