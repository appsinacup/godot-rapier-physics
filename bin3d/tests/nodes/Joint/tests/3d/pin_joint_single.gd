extends JointTest3D

# Single 3D pin joint: a box hangs from a fixed anchor and swings like a pendulum. A pin
# is a point constraint, so the hanging body's distance to the pivot must stay constant,
# it must never explode, and with damping it must settle hanging below the pivot.

var pivot: Vector3
var d0 := 2.0
var dyn: RigidBody3D

func test_name() -> String:
	return "Joint3D | Pin (single pendulum)"

func test_description() -> String:
	return "A box pinned to a fixed anchor keeps a constant distance to the pivot, never explodes, and settles below the pivot."

func test_start() -> void:
	pivot = Vector3(0, 2.0, 0)
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector3(d0, 0, 0), false, 1)

	var joint := PinJoint3D.new()
	add_child(joint)
	joint.position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	var state := {"finite": true, "bounded": true, "max_err": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector3 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_err = maxf(state.max_err, absf(p.distance_to(pivot) - d0))
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Distance to pivot stays constant (rigid pin)")
			if state.max_err > 0.3:
				m.add_test_error("max distance error was %.3f m (d0=%.1f)" % [state.max_err, d0])
			m.add_test_result(state.max_err < 0.3)

			m.add_test("Settles hanging below the pivot (stability)")
			var settled: bool = dyn.linear_velocity.length() < SETTLED_SPEED \
				and p.y < pivot.y and Vector2(p.x - pivot.x, p.z - pivot.z).length() < 0.35
			if not settled:
				m.add_test_error("v=%.3f below=%s" % [dyn.linear_velocity.length(), p.y < pivot.y])
			m.add_test_result(settled)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
