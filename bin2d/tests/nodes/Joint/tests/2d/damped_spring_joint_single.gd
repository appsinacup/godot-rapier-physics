extends JointTest2D

# Single 2D damped spring joint: a box hangs from a fixed anchor by a spring. Started
# stretched, the spring pulls it back up; damping settles the oscillation. The box must
# be pulled back toward the rest length, never explode, and come to rest.

const REST_LENGTH := 70.0

var anchor_pos: Vector2
var start_dist := 200.0
var dyn: RigidBody2D

func test_name() -> String:
	return "Joint2D | Damped spring (hang)"

func test_description() -> String:
	return "A box hung on a damped spring is pulled back toward the rest length, never explodes, and settles at equilibrium."

func test_start() -> void:
	anchor_pos = CENTER + Vector2(0, -140)
	var anchor := make_box(anchor_pos, true)
	dyn = make_box(anchor_pos + Vector2(0, start_dist), false, 3)

	var joint := DampedSpringJoint2D.new()
	add_child(joint)
	joint.position = anchor_pos
	joint.length = start_dist + 40.0
	joint.rest_length = REST_LENGTH
	joint.stiffness = 64.0
	joint.damping = 3.0
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()

	var state := {"finite": true, "bounded": true}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector2 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		if m.frame >= SETTLE_FRAME:
			var final_dist: float = p.distance_to(anchor_pos)

			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			# The spring must pull the stretched box back up (contract well below the start
			# distance) while still holding it apart (not collapsing the bodies together).
			m.add_test("Spring pulls the box back toward equilibrium")
			var restored: bool = final_dist < start_dist - 40.0 and final_dist > 25.0
			if not restored:
				m.add_test_error("final distance %.1f (start %.0f)" % [final_dist, start_dist])
			m.add_test_result(restored)

			m.add_test("Oscillation settles (stability)")
			m.add_test_result(dyn.linear_velocity.length() < SETTLED_SPEED)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
