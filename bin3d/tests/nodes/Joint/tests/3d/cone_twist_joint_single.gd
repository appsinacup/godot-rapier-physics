extends JointTest3D

# Single 3D cone-twist joint: a box hangs from a fixed anchor by a point constraint whose
# swing is limited to a cone. Starting hanging and given a shove, the point stays fixed
# (constant distance), the limited swing keeps the body in the lower hemisphere, it never
# explodes, and it damps to rest.

var pivot: Vector3
var d0 := 2.0
var dyn: RigidBody3D

func test_name() -> String:
	return "Joint3D | Cone twist (limited swing)"

func test_description() -> String:
	return "A box on a cone-twist joint keeps a constant distance to the pivot, swings within its cone, never explodes, and settles."

func test_start() -> void:
	pivot = Vector3(0, 2.0, 0)
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector3(0, -d0, 0), false, 1)

	var joint := ConeTwistJoint3D.new()
	add_child(joint)
	joint.position = pivot
	joint.set("swing_span", deg_to_rad(35.0))
	joint.set("twist_span", deg_to_rad(20.0))
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	# Shove it sideways so the swing limit is exercised.
	dyn.linear_velocity = Vector3(5.0, 0, 0)

	var state := {"finite": true, "bounded": true, "max_err": 0.0, "max_above": -1000.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector3 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_err = maxf(state.max_err, absf(p.distance_to(pivot) - d0))
		state.max_above = maxf(state.max_above, p.y - pivot.y)
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Distance to pivot stays constant (point constraint)")
			if state.max_err > 0.3:
				m.add_test_error("max distance error was %.3f m" % state.max_err)
			m.add_test_result(state.max_err < 0.3)

			m.add_test("Swing stays limited (body never rises above the pivot)")
			if state.max_above > 0.4:
				m.add_test_error("body rose %.3f m above the pivot" % state.max_above)
			m.add_test_result(state.max_above < 0.4)

			m.add_test("Settles below the pivot (stability)")
			m.add_test_result(dyn.linear_velocity.length() < SETTLED_SPEED and p.y < pivot.y)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
