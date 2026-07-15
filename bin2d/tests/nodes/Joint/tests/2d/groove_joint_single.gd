extends JointTest2D

# Single 2D groove joint: a box is constrained to slide along a vertical groove attached
# to a fixed anchor. Under gravity it slides to the bottom of the groove. The box must
# stay on the groove line (no sideways drift), never explode, and settle at the bottom.

const GROOVE_LEN := 160.0

var groove_x: float
var groove_top: Vector2
var dyn: RigidBody2D

func test_name() -> String:
	return "Joint2D | Groove (slide on rail)"

func test_description() -> String:
	return "A box on a vertical groove slides without sideways drift, never explodes, and settles at the bottom of the groove."

func test_start() -> void:
	groove_top = CENTER + Vector2(0, -120)
	groove_x = groove_top.x
	var anchor := make_box(groove_top, true)
	dyn = make_box(groove_top + Vector2(0, 40), false, 2)

	var joint := GrooveJoint2D.new()
	add_child(joint)
	joint.position = groove_top
	joint.length = GROOVE_LEN
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	var state := {"finite": true, "bounded": true, "max_drift": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector2 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_drift = maxf(state.max_drift, absf(p.x - groove_x))
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Body stays on the groove line (no sideways drift)")
			if state.max_drift > 8.0:
				m.add_test_error("max sideways drift was %.2f px" % state.max_drift)
			m.add_test_result(state.max_drift < 8.0)

			m.add_test("Slides down and settles at the groove bottom (stability)")
			var settled: bool = dyn.linear_velocity.length() < SETTLED_SPEED \
				and p.y > groove_top.y + 40.0
			m.add_test_result(settled)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
