#!/usr/bin/env python3
"""Make rover_differential stop at a goal that has nothing after it.

Upstream substitutes the home position for a missing next waypoint, then decides from the
prev->curr->next angle whether to slow down for the current one. With RD_MISS_SPD_GAIN
unset that means: whenever home lies roughly beyond the goal (within 60 degrees of straight
on), or the goal is home itself (angle undefined), the vehicle arrives at cruise speed and
only brakes inside NAV_ACC_RAD. On a boat that is a 3 m overshoot.

Run inside the container. Idempotent; keeps a .orig copy next to the source.
"""
import pathlib
import shutil
import sys

SRC = pathlib.Path("/root/PX4-Autopilot/src/modules/rover_differential/"
                   "DifferentialPosVelControl/DifferentialPosVelControl.cpp")
MARK = "// heron-twin: stop at a goal with nothing after it"

OLD = """		_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);
"""
NEW = OLD + """
		""" + MARK + """
		// (a reposition target, the last mission item, home in RTL). Upstream puts home in
		// for the missing next waypoint and then carries cruise speed through the goal
		// whenever home lies roughly beyond it, or the angle is undefined.
		if (!position_setpoint_triplet.next.valid || !PX4_ISFINITE(_waypoint_transition_angle)) {
			_waypoint_transition_angle = 0.f;
		}
"""

# Second change: "_mission_finished" is set on arriving home in Return mode and by the
# navigator when a mission ends, and nothing clears it until the next mission result. Hold
# mode (where a go-to is flown) then reads it and refuses to drive: after any completed
# return or mission the vehicle arms, accepts the go-to and sits still.
MARK2 = "// heron-twin: the finished flag belongs to Mission and Return"
OLD2 = """	if (!_mission_finished && distance_to_curr_wp > _param_nav_acc_rad.get()) {
		if (_currentState == GuidanceState::STOPPED) {"""
NEW2 = """	""" + MARK2 + """
	// only. In Hold (a go-to) the distance to the target decides.
	const bool finished = _mission_finished && (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
			      || _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	if (!finished && distance_to_curr_wp > _param_nav_acc_rad.get()) {
		if (_currentState == GuidanceState::STOPPED) {"""

text = SRC.read_text()
orig = SRC.with_suffix(".cpp.orig")
if not orig.exists():
    shutil.copy2(SRC, orig)
changed = False
for mark, old, new in ((MARK, OLD, NEW), (MARK2, OLD2, NEW2)):
    if mark in text:
        print("already there:", mark)
        continue
    if text.count(old) != 1:
        print("source does not look as expected; not patching:", mark)
        sys.exit(1)
    text = text.replace(old, new)
    changed = True
if changed:
    SRC.write_text(text)
    print("patched", SRC)
