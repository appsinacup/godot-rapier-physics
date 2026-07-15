extends JointTest2D

# Stress test: a chain of boxes hung from a fixed anchor by pin joints. Each joint pins
# two consecutive boxes at the point between them, so a link can never stretch beyond its
# rest length. The chain must not explode and must settle into a stable hanging line.

const LINKS := 5
const LINK := 56.0

var anchor: RigidBody2D
var bodies: Array[RigidBody2D] = []

func test_name() -> String:
	return "Joint2D | Pin (hanging chain)"

func test_description() -> String:
	return "A 5-link pin chain never stretches past its rest length, does not explode, and settles into a stable hanging line."

func test_start() -> void:
	var top := CENTER + Vector2(0, -150)
	anchor = make_box(top, true)
	var prev := anchor
	var prev_pos := top
	for i in range(LINKS):
		var pos := prev_pos + Vector2(0, LINK)
		var body := make_box(pos, false, i + 1)
		bodies.append(body)
		var joint := PinJoint2D.new()
		add_child(joint)
		joint.position = (prev_pos + pos) * 0.5
		joint.node_a = prev.get_path()
		joint.node_b = body.get_path()
		prev = body
		prev_pos = pos

	var state := {"finite": true, "bounded": true, "max_link": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var chain := [anchor] + bodies
		for b in chain:
			if not finite_v(b.global_position) or not finite_v(b.linear_velocity):
				state.finite = false
			if not in_bounds(b.global_position):
				state.bounded = false
		for i in range(chain.size() - 1):
			var d: float = chain[i].global_position.distance_to(chain[i + 1].global_position)
			state.max_link = maxf(state.max_link, d)
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf across the chain")
			m.add_test_result(state.finite)

			m.add_test("Whole chain stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Links never stretch past rest length (rigid pins)")
			if state.max_link > LINK + 10.0:
				m.add_test_error("max link length was %.2f (rest %.0f)" % [state.max_link, LINK])
			m.add_test_result(state.max_link < LINK + 10.0)

			m.add_test("Chain settles and hangs below the anchor (stability)")
			var settled := true
			for b in bodies:
				if b.linear_velocity.length() > SETTLED_SPEED:
					settled = false
			var hangs: bool = bodies[-1].global_position.y > anchor.global_position.y
			m.add_test_result(settled and hangs)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
