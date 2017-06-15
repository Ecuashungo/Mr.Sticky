This is the module where the states are defined.
The states are:
- init
- recovery
- homing
- go_to_point
- lookout
- grabbing


The tasks in the states are:
STATE init:
 check the availability of the components (motors, sensors, batteries, communication links, cameras, etc.)

STATE recovery:
 get an idea of where we are (check empirically which is better --> navigation, odometry or bumpers)

STATE homing
 when the robot is full (or almost, or time is up) robot needs to go home
 find the angle to the recycling led tower and go there
 in the end an automatic release procedure is done

STATE go_to_point:
 when the robot wants to go to a new zone (1-4) or wants to recenter

STATE lookout:
 search for bottles: start timer, wait a bit, turn a bit, wait, turn, until bottle has been found or timer is up
 --> next states: go to bottle, change_position

STATE grabbing:
 check if bottles sticks: turn conveyor belt, if not turn back and go forward, check again
 --> next states: lookout, go_home
