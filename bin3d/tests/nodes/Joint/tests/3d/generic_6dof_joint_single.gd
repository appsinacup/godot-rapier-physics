extends JointTest3D

# Single 3D generic-6DOF joint configured as a rigid weld: every linear and angular axis
# is locked, so the box must stay welded at its spawn pose against gravity - it must not
# drift, must not explode, and must be at rest.

var spawn: Vector3
var dyn: RigidBody3D

func test_name() -> String:
	return "Joint3D | Generic 6DOF (rigid weld)"

func test_description() -> String:
	return "A box welded with a fully-locked 6DOF joint holds its spawn pose against gravity, never drifts, and stays at rest."

func _lock_axis(joint: Generic6DOFJoint3D, axis: String) -> void:
	joint.set("linear_limit_%s/enabled" % axis, true)
	joint.set("linear_limit_%s/upper_distance" % axis, 0.0)
	joint.set("linear_limit_%s/lower_distance" % axis, 0.0)
	joint.set("angular_limit_%s/enabled" % axis, true)
	joint.set("angular_limit_%s/upper_angle" % axis, 0.0)
	joint.set("angular_limit_%s/lower_angle" % axis, 0.0)

func test_start() -> void:
	var anchor := make_box(Vector3(0, 2.0, 0), true)
	spawn = Vector3(1.3, 2.0, 0)
	dyn = make_box(spawn, false, 1)

	var joint := Generic6DOFJoint3D.new()
	add_child(joint)
	joint.position = spawn
	for axis in ["x", "y", "z"]:
		_lock_axis(joint, axis)
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	var state := {"finite": true, "bounded": true, "max_drift": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector3 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_drift = maxf(state.max_drift, p.distance_to(spawn))
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Welded body holds its spawn position against gravity")
			if state.max_drift > 0.3:
				m.add_test_error("max drift from spawn was %.3f m" % state.max_drift)
			m.add_test_result(state.max_drift < 0.3)

			m.add_test("Body is at rest (stability)")
			m.add_test_result(dyn.linear_velocity.length() < SETTLED_SPEED)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
