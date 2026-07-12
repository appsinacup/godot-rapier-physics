extends PhysicsUnitTest2D

# Regression test for the fix to `body_add_force_at_point` (constant force at a point).
# `PhysicsServer2D.body_add_constant_force(body, force, position)` interprets `position`
# as the offset from the body ORIGIN, expressed in global coordinates. The resulting
# torque must therefore be taken about the body's global center of mass:
#     torque = (position - (global_com - origin)) x force
# The pre-fix code applied the force relative to the center of mass instead of the origin,
# so with a non-zero center-of-mass offset combined with a rotation the torque was wrong.
#
# This exercises exactly that condition (custom COM + rotation) with three identical bodies:
#   * body_com : force applied AT the global center of mass -> zero torque, no spin.
#   * body_org : force applied at the body origin           -> torque about the COM.
#   * body_ref : the equivalent pure torque via add_constant_torque (reference).
# body_org must spin exactly like body_ref (same torque, inertia and integrator), which is
# inertia- and timing-independent. The pre-fix code would instead spin body_com and leave
# body_org matching a zero-torque body - the opposite of correct.

var simulation_duration := 10.0
var dt := 1.0 / 60.0

var mass := 2.0
var local_com := Vector2(3, 0)
var body_rotation := PI / 2.0
var force := Vector2(100, 0)

var body_com: RID
var body_org: RID
var body_ref: RID
var apply_frame := 2
var check_frame := 42

func test_description() -> String:
	return """Checks that a constant force applied at a point produces the correct torque
	about the global center of mass, for a body with a custom center of mass and a rotation.
	"""

func test_name() -> String:
	return "RigidBody2D | add_force_at_point torque about center of mass"

func _make_body(p_position: Vector2) -> RID:
	var rid := PhysicsServer2D.body_create()
	PhysicsServer2D.body_set_space(rid, get_viewport().world_2d.space)
	PhysicsServer2D.body_set_mode(rid, PhysicsServer2D.BODY_MODE_RIGID)
	# Isolate from the other tests sharing this space: no collisions at all, so only the
	# applied force drives the body.
	PhysicsServer2D.body_set_collision_layer(rid, 0)
	PhysicsServer2D.body_set_collision_mask(rid, 0)

	var shape := PhysicsServer2D.rectangle_shape_create()
	PhysicsServer2D.shape_set_data(shape, Vector2(20, 20))
	PhysicsServer2D.body_add_shape(rid, shape)

	PhysicsServer2D.body_set_param(rid, PhysicsServer2D.BODY_PARAM_GRAVITY_SCALE, 0.0)
	PhysicsServer2D.body_set_param(rid, PhysicsServer2D.BODY_PARAM_MASS, mass)
	PhysicsServer2D.body_set_param(rid, PhysicsServer2D.BODY_PARAM_CENTER_OF_MASS, local_com)

	var xform := Transform2D(body_rotation, p_position)
	PhysicsServer2D.body_set_state(rid, PhysicsServer2D.BODY_STATE_TRANSFORM, xform)
	return rid

func test_start() -> void:
	body_com = _make_body(Vector2(200, 200))
	body_org = _make_body(Vector2(450, 200))
	body_ref = _make_body(Vector2(700, 200))

	# The center of mass offset relative to the origin, in global orientation.
	var global_com_offset := Transform2D(body_rotation, Vector2.ZERO).basis_xform(local_com)
	# The torque a force at the origin must produce about the center of mass.
	var expected_torque := (Vector2.ZERO - global_com_offset).cross(force)

	var checks := func(_p_target, p_monitor: GenericManualMonitor):
		if p_monitor.frame == apply_frame:
			# body_com: apply the force exactly at the global center of mass -> zero torque.
			PhysicsServer2D.body_add_constant_force(body_com, force, global_com_offset)
			# body_org: apply the same force at the origin -> torque about the COM.
			PhysicsServer2D.body_add_constant_force(body_org, force, Vector2.ZERO)
			# body_ref: the equivalent pure torque, no linear force.
			PhysicsServer2D.body_add_constant_torque(body_ref, expected_torque)

		if p_monitor.frame == check_frame:
			var ds_com := PhysicsServer2D.body_get_direct_state(body_com)
			var ds_org := PhysicsServer2D.body_get_direct_state(body_org)
			var ds_ref := PhysicsServer2D.body_get_direct_state(body_ref)
			var elapsed := (check_frame - apply_frame) * dt

			# Force applied at the center of mass must not induce any rotation.
			p_monitor.add_test("Force at the center of mass produces no rotation")
			var com_spin_ok: bool = abs(ds_com.angular_velocity) < 0.02
			if not com_spin_ok:
				p_monitor.add_test_error("Expected ~0 angular velocity, got %f" % ds_com.angular_velocity)
			p_monitor.add_test_result(com_spin_ok)

			# Force at the origin must spin the body exactly like the equivalent pure torque.
			p_monitor.add_test("Force at the origin produces the torque about the center of mass")
			var reference_nonzero: bool = abs(ds_ref.angular_velocity) > 0.1
			var torque_ok: bool = reference_nonzero and Utils.f_equals(ds_org.angular_velocity, ds_ref.angular_velocity, abs(ds_ref.angular_velocity) * 0.02)
			if not torque_ok:
				p_monitor.add_test_error("Expected angular velocity ~%f (pure torque), got %f" % [ds_ref.angular_velocity, ds_org.angular_velocity])
			p_monitor.add_test_result(torque_ok)

			# The point of application must not change the linear response: both forced bodies
			# receive the same net force, so their linear velocities must match and equal
			# force / mass * time.
			p_monitor.add_test("Linear response is independent of the application point")
			var expected_lin: Vector2 = force / mass * elapsed
			var lin_ok: bool = Utils.vec2_equals(ds_com.linear_velocity, ds_org.linear_velocity, 0.01) \
				and Utils.vec2_equals(ds_com.linear_velocity, expected_lin, expected_lin.length() * 0.15)
			if not lin_ok:
				p_monitor.add_test_error("Linear velocities: com %v, org %v, expected %v" % [ds_com.linear_velocity, ds_org.linear_velocity, expected_lin])
			p_monitor.add_test_result(lin_ok)

			PhysicsServer2D.free_rid(body_com)
			PhysicsServer2D.free_rid(body_org)
			PhysicsServer2D.free_rid(body_ref)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
