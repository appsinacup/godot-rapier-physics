extends JointTest2D

# 2D pin joint softness sweep: four identical heavy bodies hang from pins whose softness
# ranges 0 -> 16. Softness makes the constraint compliant, so a softer pin should let the
# body sink further below the pivot under the same load. We assert the sink distance is
# ordered by softness (softer never stiffer), stays bounded, and never explodes. The four
# measured sink distances are printed so the exact behaviour is visible.

const SOFTNESS := [0.0, 4.0, 8.0, 16.0]
const D0 := 60.0
const MASS := 12.0

var pivots: Array[Vector2] = []
var bodies: Array[RigidBody2D] = []

func test_name() -> String:
	return "Joint2D | Pin softness sweep (0..16)"

func test_description() -> String:
	return "Heavier softness makes a loaded pin more compliant; the body sinks further below the pivot. Sink distance is ordered by softness and stays bounded."

func test_start() -> void:
	var base_x := Global.WINDOW_SIZE.x * 0.2
	var step_x := Global.WINDOW_SIZE.x * 0.2
	for i in range(SOFTNESS.size()):
		var pivot := Vector2(base_x + step_x * i, 120.0)
		pivots.append(pivot)
		var anchor := make_box(pivot, true)
		var body := make_box(pivot + Vector2(0, D0), false, i)
		body.mass = MASS
		bodies.append(body)
		var joint := PinJoint2D.new()
		add_child(joint)
		joint.position = pivot
		joint.softness = SOFTNESS[i]
		joint.node_a = anchor.get_path()
		joint.node_b = body.get_path()

	var state := {"finite": true, "bounded": true}
	var checks := func(_t, m: GenericManualMonitor):
		for b in bodies:
			if not finite_v(b.global_position) or not finite_v(b.linear_velocity):
				state.finite = false
			if not in_bounds(b.global_position):
				state.bounded = false
		if m.frame >= SETTLE_FRAME:
			var sink: Array[float] = []
			for i in range(bodies.size()):
				sink.append(bodies[i].global_position.distance_to(pivots[i]))

			m.add_test("No NaN/Inf in any body")
			m.add_test_result(state.finite)

			m.add_test("All bodies stay bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Sink distance is non-decreasing with softness [%s]" % [", ".join(sink.map(func(v): return "%.1f" % v))])
			var ordered := true
			for i in range(sink.size() - 1):
				if sink[i] > sink[i + 1] + 3.0:   # allow tiny solver noise
					ordered = false
			m.add_test_result(ordered)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
