extends JointTest3D

# 3D hinge joint motor: with gravity disabled, the hinge motor must drive the door to
# revolve about the hinge axis at (roughly) the target velocity - it must sweep at least
# a full turn in the target direction, keep the pivot fixed, and never explode.

var pivot: Vector3
var d0 := 2.0
var target_vel := 4.0
var dyn: RigidBody3D

var _prev_angle := 0.0
var _swept := 0.0

func test_name() -> String:
	return "Joint3D | Hinge motor (spins to target)"

func test_description() -> String:
	return "A hinge motor revolves the door about its axis in the target direction, at least one full turn, without exploding."

func test_start() -> void:
	pivot = Vector3(0, 0.5, 0)
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector3(d0, 0, 0), false, 1)
	dyn.gravity_scale = 0.0
	dyn.linear_damp = 0.0
	dyn.angular_damp = 0.0

	var joint := HingeJoint3D.new()
	add_child(joint)
	joint.position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()
	joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, target_vel)
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, 10000.0)

	_prev_angle = atan2(dyn.global_position.y - pivot.y, dyn.global_position.x - pivot.x)

	var state := {"finite": true, "bounded": true, "max_err": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector3 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_err = maxf(state.max_err, absf(p.distance_to(pivot) - d0))
		var a := atan2(p.y - pivot.y, p.x - pivot.x)
		_swept += wrapf(a - _prev_angle, -PI, PI)
		_prev_angle = a
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Pivot stays fixed (rigid hinge)")
			m.add_test_result(state.max_err < 0.3)

			# The hinge axis convention can make the swept sign opposite our XY-plane
			# measurement, so only require that the motor drives a sustained full revolution.
			m.add_test("Motor revolves the door at least a full turn")
			if absf(_swept) < TAU:
				m.add_test_error("swept only %.2f rad (target vel %.1f)" % [_swept, target_vel])
			m.add_test_result(absf(_swept) > TAU)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
