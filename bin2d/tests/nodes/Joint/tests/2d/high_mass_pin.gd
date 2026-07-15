extends JointTest2D

# Heavy-mass stability: a very heavy box hangs from a light static anchor by a rigid pin.
# A large mass ratio is a classic solver stressor - it must not blow the constraint up.
# The pin must hold (distance stays constant), nothing may explode, and it must settle.

const HEAVY_MASS := 500.0
var pivot: Vector2
var d0 := 90.0
var dyn: RigidBody2D

func test_name() -> String:
	return "Joint2D | Pin high mass (500kg)"

func test_description() -> String:
	return "A 500kg box on a rigid pin holds its distance to the pivot, never explodes, and settles hanging below it."

func test_start() -> void:
	pivot = CENTER + Vector2(0, -100)
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector2(d0, 0), false, 1)
	dyn.mass = HEAVY_MASS

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
			m.add_test("No NaN/Inf under heavy load")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Rigid pin holds despite the heavy mass")
			if state.max_err > 18.0:
				m.add_test_error("max distance error was %.2f px" % state.max_err)
			m.add_test_result(state.max_err < 18.0)

			m.add_test("Settles hanging below the pivot (stability)")
			m.add_test_result(dyn.linear_velocity.length() < SETTLED_SPEED and p.y > pivot.y)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
