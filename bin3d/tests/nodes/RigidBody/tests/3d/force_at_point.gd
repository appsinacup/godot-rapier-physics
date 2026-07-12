extends PhysicsUnitTest3D

# Regression test for the fix to `body_add_force_at_point` (constant force at a point).
# `PhysicsServer3D.body_add_constant_force(body, force, position)` interprets `position`
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
var local_com := Vector3(3, 0, 0)
var body_rotation := PI / 2.0
var force := Vector3(100, 0, 0)

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
	return "RigidBody3D | add_force_at_point torque about center of mass"

func _make_body(p_position: Vector3) -> RID:
	var rid := PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_space(rid, get_world_3d().space)
	PhysicsServer3D.body_set_mode(rid, PhysicsServer3D.BODY_MODE_RIGID)
	# Isolate from the other tests sharing this space: no collisions at all, so only the
	# applied force drives the body.
	PhysicsServer3D.body_set_collision_layer(rid, 0)
	PhysicsServer3D.body_set_collision_mask(rid, 0)

	var shape := PhysicsServer3D.box_shape_create()
	PhysicsServer3D.shape_set_data(shape, Vector3(10, 10, 10))
	PhysicsServer3D.body_add_shape(rid, shape)

	PhysicsServer3D.body_set_param(rid, PhysicsServer3D.BODY_PARAM_GRAVITY_SCALE, 0.0)
	PhysicsServer3D.body_set_param(rid, PhysicsServer3D.BODY_PARAM_MASS, mass)
	PhysicsServer3D.body_set_param(rid, PhysicsServer3D.BODY_PARAM_CENTER_OF_MASS, local_com)

	var xform := Transform3D(Basis(Vector3(0, 0, 1), body_rotation), p_position)
	PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM, xform)
	return rid

func test_start() -> void:
	# The center of mass offset relative to the origin, in global orientation.
	var global_com_offset := Basis(Vector3(0, 0, 1), body_rotation) * local_com
	# The torque a force at the origin must produce about the center of mass.
	var expected_torque := (Vector3.ZERO - global_com_offset).cross(force)

	var checks := func(_p_target, p_monitor: GenericManualMonitor):
		# Create the bodies from inside the monitor callback: at test_start() the runner's
		# SubViewport world is not yet initialized, so get_viewport().world_3d can be null.
		if p_monitor.frame == 1:
			body_com = _make_body(Vector3(0, 0, 0))
			body_org = _make_body(Vector3(50, 0, 0))
			body_ref = _make_body(Vector3(100, 0, 0))

		if p_monitor.frame == apply_frame:
			# body_com: apply the force exactly at the global center of mass -> zero torque.
			PhysicsServer3D.body_add_constant_force(body_com, force, global_com_offset)
			# body_org: apply the same force at the origin -> torque about the COM.
			PhysicsServer3D.body_add_constant_force(body_org, force, Vector3.ZERO)
			# body_ref: the equivalent pure torque, no linear force.
			PhysicsServer3D.body_add_constant_torque(body_ref, expected_torque)

		if p_monitor.frame == check_frame:
			var ds_com := PhysicsServer3D.body_get_direct_state(body_com)
			var ds_org := PhysicsServer3D.body_get_direct_state(body_org)
			var ds_ref := PhysicsServer3D.body_get_direct_state(body_ref)
			var elapsed := (check_frame - apply_frame) * dt

			# Force applied at the center of mass must not induce any rotation.
			p_monitor.add_test("Force at the center of mass produces no rotation")
			var com_spin_ok: bool = ds_com.angular_velocity.length() < 0.02
			if not com_spin_ok:
				p_monitor.add_test_error("Expected ~0 angular velocity, got %v" % ds_com.angular_velocity)
			p_monitor.add_test_result(com_spin_ok)

			# Force at the origin must spin the body exactly like the equivalent pure torque.
			p_monitor.add_test("Force at the origin produces the torque about the center of mass")
			var reference_nonzero: bool = ds_ref.angular_velocity.length() > 0.1
			var torque_ok: bool = reference_nonzero and Utils.vec3_equals(ds_org.angular_velocity, ds_ref.angular_velocity, ds_ref.angular_velocity.length() * 0.02)
			if not torque_ok:
				p_monitor.add_test_error("Expected angular velocity ~%v (pure torque), got %v" % [ds_ref.angular_velocity, ds_org.angular_velocity])
			p_monitor.add_test_result(torque_ok)

			# The point of application must not change the linear response: both forced bodies
			# receive the same net force, so their linear velocities must match and equal
			# force / mass * time.
			p_monitor.add_test("Linear response is independent of the application point")
			var expected_lin: Vector3 = force / mass * elapsed
			var lin_ok: bool = Utils.vec3_equals(ds_com.linear_velocity, ds_org.linear_velocity, 0.01) \
				and Utils.vec3_equals(ds_com.linear_velocity, expected_lin, expected_lin.length() * 0.15)
			if not lin_ok:
				p_monitor.add_test_error("Linear velocities: com %v, org %v, expected %v" % [ds_com.linear_velocity, ds_org.linear_velocity, expected_lin])
			p_monitor.add_test_result(lin_ok)

			PhysicsServer3D.free_rid(body_com)
			PhysicsServer3D.free_rid(body_org)
			PhysicsServer3D.free_rid(body_ref)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
