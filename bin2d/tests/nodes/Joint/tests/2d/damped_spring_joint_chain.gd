extends JointTest2D

# Stress test: a chain of boxes connected by damped springs, hung from a fixed anchor.
# The springy chain must not explode or oscillate forever - it must settle into a stable
# hanging column.

const LINKS := 4
const REST := 55.0

var anchor: RigidBody2D
var bodies: Array[RigidBody2D] = []

func test_name() -> String:
	return "Joint2D | Damped spring (chain)"

func test_description() -> String:
	return "A 4-link damped-spring chain does not explode, stays bounded, and settles into a stable hanging column."

func test_start() -> void:
	var top := CENTER + Vector2(0, -150)
	anchor = make_box(top, true)
	var prev := anchor
	var prev_pos := top
	for i in range(LINKS):
		var pos := prev_pos + Vector2(0, REST)
		var body := make_box(pos, false, i + 1)
		bodies.append(body)
		var joint := DampedSpringJoint2D.new()
		add_child(joint)
		joint.position = prev_pos
		joint.length = REST + 30.0
		joint.rest_length = REST
		joint.stiffness = 200.0
		joint.damping = 5.0
		joint.node_a = prev.get_path()
		joint.node_b = body.get_path()
		prev = body
		prev_pos = pos

	var state := {"finite": true, "bounded": true}
	var checks := func(_t, m: GenericManualMonitor):
		var chain := [anchor] + bodies
		for b in chain:
			if not finite_v(b.global_position) or not finite_v(b.linear_velocity):
				state.finite = false
			if not in_bounds(b.global_position):
				state.bounded = false
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf across the chain")
			m.add_test_result(state.finite)

			m.add_test("Whole chain stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Chain settles and hangs below the anchor (stability)")
			var settled := true
			for b in bodies:
				if b.linear_velocity.length() > SETTLED_SPEED:
					settled = false
			var hangs: bool = bodies[-1].global_position.y > anchor.global_position.y
			m.add_test_result(settled and hangs)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
