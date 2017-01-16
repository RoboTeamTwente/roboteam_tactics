# RoboTeam Tactics
Secret smart package!

## Conventions

When specifying which robot a skill or condition should work on, use the Int: `ROBOT_ID`.
When specifying for a conditions which team to work on, use the boolean: `our_team`.

## Dependencies
- unique\_identifier

Command:
sudo apt install ros-kinetic-unique-identifier

## Compiling
1. run catkin\_make
2. ???
3. Profit!

Should Just Work™. If not, contact a RTT member. *crosses fingers*

## How do I add a Behavior3 project?
1. Create a B3 project in the src/trees/projects folder with the Behavior3 Editor. Make sure it contains at least one valid tree. For example: a "KickTest" tree, with only a "Kick" node with name "Kick_A" in it.
2. Add it to the `CMakeLists.txt` there.
3. Run `refresh_b3_projects.sh`. This can take a minute or 2.
4. All currently known trees will be printed afterwards. Make sure your new project & trees are in there.


