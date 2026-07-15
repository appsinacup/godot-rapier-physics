extends JointTest3D

# Stress test: a chain hung from a fixed anchor by generic-6DOF joints with their linear
# axes locked (angular free), so each acts like a point link. The chain must not stretch
# past its rest length, must not explode, and must settle hanging.

const LINKS := 5
const LINK := 1.1

var anchor: RigidBody3D
var bodies: Array[RigidBody3D] = []

func test_name() -> String:
	return "Joint3D | Generic 6DOF (hanging chain)"

func test_description() -> String:
	return "A 5-link 6DOF chain (linear axes locked) never over-stretches, does not explode, and settles into a stable hanging line."

func _lock_linear(joint: Generic6DOFJoint3D) -> void:
	for axis in ["x", "y", "z"]:
		joint.set("linear_limit_%s/enabled" % axis, true)
		joint.set("linear_limit_%s/upper_distance" % axis, 0.0)
		joint.set("linear_limit_%s/lower_distance" % axis, 0.0)

func test_start() -> void:
	var top := Vector3(0, 2.8, 0)
	anchor = make_box(top, true)
	var prev := anchor
	var prev_pos := top
	for i in range(LINKS):
		var pos := prev_pos + Vector3(0, -LINK, 0)
		var body := make_box(pos, false, i + 1)
		bodies.append(body)
		var joint := Generic6DOFJoint3D.new()
		add_child(joint)
		joint.position = (prev_pos + pos) * 0.5
		_lock_linear(joint)
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

			m.add_test("Links never over-stretch")
			if state.max_link > LINK + 0.35:
				m.add_test_error("max link length was %.3f (rest %.1f)" % [state.max_link, LINK])
			m.add_test_result(state.max_link < LINK + 0.35)

			m.add_test("Chain settles and hangs below the anchor (stability)")
			var settled := true
			for b in bodies:
				if b.linear_velocity.length() > SETTLED_SPEED:
					settled = false
			var hangs: bool = bodies[-1].global_position.y < anchor.global_position.y
			m.add_test_result(settled and hangs)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
