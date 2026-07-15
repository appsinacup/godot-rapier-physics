extends JointTest2D

# 2D pin joint motor: with gravity disabled, the pin's angular motor must drive the body
# to revolve around the pivot at (roughly) the target velocity - i.e. the motor actually
# "goes somewhere". The body must sweep at least a full revolution in the motor direction,
# keep a constant distance to the pivot, and never explode.

var pivot: Vector2
var d0 := 100.0
var target_vel := 4.0   # rad/s
var dyn: RigidBody2D

var _prev_angle := 0.0
var _swept := 0.0

func test_name() -> String:
	return "Joint2D | Pin motor (spins to target)"

func test_description() -> String:
	return "A pin joint's angular motor revolves the body around the pivot in the target direction, at least one full turn, without exploding."

func test_start() -> void:
	pivot = CENTER
	var anchor := make_box(pivot, true)
	dyn = make_box(pivot + Vector2(d0, 0), false, 1)
	# Isolate the motor: no gravity, no damping fighting it.
	dyn.gravity_scale = 0.0
	dyn.linear_damp = 0.0
	dyn.angular_damp = 0.0

	var joint := PinJoint2D.new()
	add_child(joint)
	joint.position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = dyn.get_path()
	joint.motor_enabled = true
	joint.motor_target_velocity = target_vel

	_prev_angle = (dyn.global_position - pivot).angle()

	var state := {"finite": true, "bounded": true, "max_err": 0.0}
	var checks := func(_t, m: GenericManualMonitor):
		var p: Vector2 = dyn.global_position
		if not finite_v(p) or not finite_v(dyn.linear_velocity):
			state.finite = false
		if not in_bounds(p):
			state.bounded = false
		state.max_err = maxf(state.max_err, absf(p.distance_to(pivot) - d0))
		# Accumulate the (unwrapped) revolution angle.
		var a := (p - pivot).angle()
		_swept += wrapf(a - _prev_angle, -PI, PI)
		_prev_angle = a
		if m.frame >= SETTLE_FRAME:
			m.add_test("No NaN/Inf in body state")
			m.add_test_result(state.finite)

			m.add_test("Body stays bounded (no explosion)")
			m.add_test_result(state.bounded)

			m.add_test("Distance to pivot stays constant (rigid pin)")
			m.add_test_result(state.max_err < 12.0)

			m.add_test("Motor revolves the body at least a full turn")
			if absf(_swept) < TAU:
				m.add_test_error("swept only %.2f rad (target vel %.1f)" % [_swept, target_vel])
			m.add_test_result(absf(_swept) > TAU)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, MAX_DURATION)
