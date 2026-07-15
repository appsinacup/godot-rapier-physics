extends JointTest2D

# Single 2D pin joint: a box hangs from a fixed anchor box and swings like a pendulum.
# A pin is a point constraint, so the hanging body's distance to the pivot must stay
# constant (rigid), it must never explode, and with damping it must settle hanging
# straight below the pivot.

var pivot: Vector2
var d0 := 100.0
var dyn: RigidBody2D

func test_name() -> String:
	return "Joint2D | Pin (single pendulum)"

func test_description() -> String:
	return "A box pinned to a fixed anchor keeps a constant distance to the pivot, never explodes, and settles below the pivot."

func test_start() -> void:
	pivot = CENTER + Vector2(0, -110)
	var anchor := make_box(pivot, true)
	# Start displaced horizontally so it swings through a large arc.
	dyn = make_box(pivot + Vector2(d0, 0), false, 1)

	var joint := PinJoint2D.new()
	add_child(joint)
	joint.position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	var state := {"finite": true, "bounded": true, "max_err": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector2 = dyn.global_position
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
			if state.max_err > 12.0:
				m.add_test_error("max distance error was %.2f px (d0=%.0f)" % [state.max_err, d0])
			m.add_test_result(state.max_err < 12.0)

			m.add_test("Settles hanging below the pivot (stability)")
			var settled: bool = dyn.linear_velocity.length() < SETTLED_SPEED \
				and absf(p.x - pivot.x) < 14.0 and p.y > pivot.y
			if not settled:
				m.add_test_error("v=%.2f x_off=%.2f" % [dyn.linear_velocity.length(), p.x - pivot.x])
			m.add_test_result(settled)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
