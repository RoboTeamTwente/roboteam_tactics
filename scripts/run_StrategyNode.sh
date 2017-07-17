#!/bin/bash

rosrun roboteam_tactics StrategyNode "$@"
while [ $? -eq 42 ]; do
  rosrun roboteam_tactics StrategyNode "$@"
done
