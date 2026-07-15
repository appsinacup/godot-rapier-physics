extends JointTest3D

# Single 3D hinge joint: a box hangs from a fixed anchor and may only rotate about the
# hinge axis. The pivot point stays fixed (constant distance), the motion stays in the
# plane perpendicular to the hinge axis, it never explodes, and it settles below the pivot.

var pivot: Vector3
var d0 := 2.0
var dyn: RigidBody3D

func test_name() -> String:
	return "Joint3D | Hinge (single door)"

func test_description() -> String:
	return "A box on a hinge keeps a constant distance to the pivot, swings in the hinge plane, never explodes, and settles below the pivot."

func test_start() -> void:
	pivot = Vector3(0, 2.0, 0)
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector3(d0, 0, 0), false, 1)

	var joint := HingeJoint3D.new()
	add_child(joint)
	joint.position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	var state := {"finite": true, "bounded": true, "max_err": 0.0, "max_plane": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector3 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_err = maxf(state.max_err, absf(p.distance_to(pivot) - d0))
		# Default hinge axis is the joint's local Z, so motion stays in the XY plane (z fixed).
		state.max_plane = maxf(state.max_plane, absf(p.z - pivot.z))
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Distance to pivot stays constant (rigid hinge)")
			if state.max_err > 0.3:
				m.add_test_error("max distance error was %.3f m" % state.max_err)
			m.add_test_result(state.max_err < 0.3)

			m.add_test("Motion stays in the hinge plane")
			if state.max_plane > 0.3:
				m.add_test_error("max out-of-plane offset was %.3f m" % state.max_plane)
			m.add_test_result(state.max_plane < 0.3)

			m.add_test("Settles below the pivot (stability)")
			m.add_test_result(dyn.linear_velocity.length() < SETTLED_SPEED and p.y < pivot.y)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
