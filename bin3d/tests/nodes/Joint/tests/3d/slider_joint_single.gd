extends JointTest3D

# Single 3D slider joint: a box may only slide along the slider axis (horizontal X). It is
# given a push along the rail; gravity (perpendicular to the rail) must be resisted by the
# joint so the box never leaves the rail line, never explodes, and damps to a stop.

const RAIL_Y := 1.5

var start_x := 0.5
var dyn: RigidBody3D

func test_name() -> String:
	return "Joint3D | Slider (rail)"

func test_description() -> String:
	return "A box on a horizontal slider stays on the rail line against gravity, never explodes, and damps to a stop."

func test_start() -> void:
	var anchor := make_box(Vector3(-1.5, RAIL_Y, 0), true)
	dyn = make_box(Vector3(start_x, RAIL_Y, 0), false, 1)

	var joint := SliderJoint3D.new()
	add_child(joint)
	joint.position = Vector3(0, RAIL_Y, 0)
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()
	# Allow a wide travel along the rail so the push actually slides the body.
	joint.set("linear_limit/upper_distance", 5.0)
	joint.set("linear_limit/lower_distance", -5.0)

	# Push along the rail.
	dyn.linear_velocity = Vector3(4.0, 0, 0)

	var state := {"finite": true, "bounded": true, "max_perp": 0.0, "max_slide": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector3 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		# Perpendicular deviation from the rail line (y = RAIL_Y, z = 0).
		state.max_perp = maxf(state.max_perp, Vector2(p.y - RAIL_Y, p.z).length())
		state.max_slide = maxf(state.max_slide, absf(p.x - start_x))
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Body stays on the rail against gravity (no perpendicular drift)")
			if state.max_perp > 0.3:
				m.add_test_error("max perpendicular drift was %.3f m" % state.max_perp)
			m.add_test_result(state.max_perp < 0.3)

			m.add_test("Body actually slid along the rail then damped to a stop (stability)")
			m.add_test_result(state.max_slide > 0.5 and dyn.linear_velocity.length() < SETTLED_SPEED)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
