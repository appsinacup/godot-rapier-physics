extends JointTest3D

# Heavy-mass stability: a very heavy box hangs from a light static anchor by a rigid pin.
# The large mass ratio must not blow up the constraint - the pin must hold, nothing may
# explode, and it must settle.

const HEAVY_MASS := 500.0
var pivot: Vector3
var d0 := 2.0
var dyn: RigidBody3D

func test_name() -> String:
	return "Joint3D | Pin high mass (500kg)"

func test_description() -> String:
	return "A 500kg box on a rigid pin holds its distance to the pivot, never explodes, and settles hanging below it."

func test_start() -> void:
	pivot = Vector3(0, 2.0, 0)
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector3(d0, 0, 0), false, 1)
	dyn.mass = HEAVY_MASS

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
			m.add_test("No NaN/Inf under heavy load")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Rigid pin holds despite the heavy mass")
			if state.max_err > 0.4:
				m.add_test_error("max distance error was %.3f m" % state.max_err)
			m.add_test_result(state.max_err < 0.4)

			m.add_test("Settles hanging below the pivot (stability)")
			m.add_test_result(dyn.linear_velocity.length() < SETTLED_SPEED and p.y < pivot.y)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
