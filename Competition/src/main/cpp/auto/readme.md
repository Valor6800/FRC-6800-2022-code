# File-Based Auto Functionanilty and Usage

Contents
========
- [Auto](#auto)
    - [Auto commands](#auto-commands)
        - [Trajectory](#trajectory)
        - [State](#state)
        - [Time](#time)
        - [Reset odom](#reset-odom)
        - [Action](#action)
- [Points](#points)

## Auto

Auto files are composed of lines of commands in the following format:
```
command_name,command_arg_1,command_arg_2,...
```
Example auto file:
```
reset_odom,start
trajectory,start,bugs
time,500
state,flywheel,FLYWHEEl_DEFAULT
```
Note that there are no spaces. 

Auto files are located in the roboRIO under `~/auto_csvs/`. (_In the code, this is `home/lvuser/auto_csvs/`_)

Autos, as well as point and action files, are CSV files. 

## Auto Commands

Auto command documentation notation
- `<>` means it is a required field
- `[]` means it is an optional field
- `""` means a choice is a literal

### Trajectory
Constructs a trajectory between 2+ (see below) points.

Syntax: `trajectory,<start_point>,<end_point>,<rotation>,["normal"/"reversed"]`

Trajectories placed in succession will merge into one trajectory. As such, the robot may not land exactly on the inbetween points specified. Note that this merging occurs only between trajectories with the same config. As an example,
```
trajectory,p1,p2,0
trajectory,p2,p1,0,reversed
```
would result in two different trajectories being created. 

The trajectory config is normal by default.

### State
Sets the state for a subsystem.

Syntax: `state,<subsystem_name>,<state_name>`

<!-- @TODO Write out the available subsystems and the available states -->

### Time
Creates a delay between actions

Syntax: `time,<wait_time>`

wait_time is in milliseconds

### Reset odom
Resets odometry to a specified pose

Syntax: `reset_odom,<intial_point>,<initial_heading>`

### Action
Runs an action

Syntax: `action,<action_name>`

`action` is a special command. It's essentially, in itself, an auto. Actions can use all the commands found in a regular auto, including other actions. As such, they can be nested (_**Nesting is untested**_). The action files use the exact same syntax as a regular auto file. The only difference between an action and a regular auto is that actions are unavailable to run and autos cannot be run within another auto.

Actions are found in the `~/actions/` (_`home/lvuser/actions/` in code_) folder. 

An action's name is just its filename, without the ending. So for example, if an action file is name `testAction.csv`, its name as written in the `action` command would be `testAction`.

## Points

Point files are locating in the roboRIO home folder. 

Point files are made of lines of the following:
```
point_name,point_x,point_y
```
An example point file:
```
foo,5.5,2
bar_1,6,6.9
0baz,1,3
```
Note that there are no spaces.

Point files contain all the points used in autos and actions. Point files can be updated independently of autos, as autos use the point names.