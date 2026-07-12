extends Node3D

func _ready():
	test_body_create()
	test_body_empty()
	test_body()
	test_body_collision_exception_invalid()
	test_body_invalid()
	test_body_center_of_mass_3d()

func test_body_collision_exception_invalid():
	print("test_body_collision_exception_invalid")
	PhysicsServer3D.body_add_collision_exception(RID(), RID())
	PhysicsServer3D.body_remove_collision_exception(RID(), RID())

# Robustness: the physics server must never crash when given invalid RIDs or
# out-of-bounds shape indices. Godot's built-in server fatals on these; ours must
# gracefully ignore them and return safe defaults.
func test_body_invalid():
	print("test_body_invalid")

	# Out-of-bounds shape indices on a valid body that has no shapes.
	var body_rid = PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_space(body_rid, get_viewport().world_3d.space)
	assert(PhysicsServer3D.body_get_shape_count(body_rid) == 0)
	for idx in [0, 1, 100]:
		assert(PhysicsServer3D.body_get_shape(body_rid, idx) == RID())
		assert(PhysicsServer3D.body_get_shape_transform(body_rid, idx) == Transform3D())
		PhysicsServer3D.body_set_shape(body_rid, idx, RID())
		PhysicsServer3D.body_set_shape_transform(body_rid, idx, Transform3D())
		PhysicsServer3D.body_set_shape_disabled(body_rid, idx, true)
		PhysicsServer3D.body_remove_shape(body_rid, idx)
	assert(PhysicsServer3D.body_get_shape_count(body_rid) == 0)

	# Operations on a freed body RID must not crash.
	PhysicsServer3D.free_rid(body_rid)
	PhysicsServer3D.body_set_mode(body_rid, PhysicsServer3D.BODY_MODE_RIGID)
	assert(PhysicsServer3D.body_get_direct_state(body_rid) == null)
	assert(!PhysicsServer3D.body_get_space(body_rid).is_valid())
	assert(PhysicsServer3D.body_get_shape_count(body_rid) == 0)

	# A never-registered RID must not crash.
	var bogus = RID()
	PhysicsServer3D.body_set_mode(bogus, PhysicsServer3D.BODY_MODE_RIGID)
	PhysicsServer3D.body_set_shape(bogus, 0, RID())
	PhysicsServer3D.body_remove_shape(bogus, 0)
	assert(PhysicsServer3D.body_get_shape_count(bogus) == 0)
	assert(PhysicsServer3D.body_get_shape(bogus, 0) == RID())
	assert(PhysicsServer3D.body_get_direct_state(bogus) == null)

func test_body_create():
	print("test_body_create")
	var body = PhysicsServer3D.body_create()
	var space = get_viewport().world_3d.space
	assert(body.is_valid())
	assert(!PhysicsServer3D.body_get_space(body).is_valid())
	PhysicsServer3D.body_set_space(body, RID())
	assert(!PhysicsServer3D.body_get_space(body).is_valid())
	PhysicsServer3D.body_set_space(body, space)
	assert(PhysicsServer3D.body_get_space(body) == space)
	PhysicsServer3D.body_set_space(body, RID())
	assert(!PhysicsServer3D.body_get_space(body).is_valid())
	PhysicsServer3D.body_set_space(body, space)
	PhysicsServer3D.free_rid(body)
	assert(!PhysicsServer3D.body_get_space(body).is_valid())
	PhysicsServer3D.free_rid(body)
	PhysicsServer3D.free_rid(RID())

func test_body_empty():
	print("test_body_empty")
	var body_rid = RID()

	var shape_rid = RID()
	var new_transform = Transform3D()
	var force = Vector3(0, 0, 0)
	var impulse = Vector3(0, 0, 0)
	var callback = Callable()
	var excepted_body_rid = RID()
	var space_rid = RID()
	var axis_velocity = Vector3(1, 2, 3)
	var priority = 1.0
	var torque = Vector3(0, 0, 10)

	PhysicsServer3D.body_add_constant_central_force(body_rid, force)
	PhysicsServer3D.body_add_constant_force(body_rid, force)
	PhysicsServer3D.body_add_constant_torque(body_rid, torque)
	PhysicsServer3D.body_add_shape(body_rid, shape_rid, new_transform)
	PhysicsServer3D.body_apply_central_force(body_rid, force)
	PhysicsServer3D.body_apply_central_impulse(body_rid, impulse)
	PhysicsServer3D.body_apply_force(body_rid, force, impulse)
	PhysicsServer3D.body_apply_impulse(body_rid, impulse, force)
	PhysicsServer3D.body_apply_torque(body_rid, torque)
	PhysicsServer3D.body_apply_torque_impulse(body_rid, torque)
	PhysicsServer3D.body_attach_object_instance_id(body_rid, 0)
	PhysicsServer3D.body_clear_shapes(body_rid)

	var collision_layer = PhysicsServer3D.body_get_collision_layer(body_rid)
	assert(collision_layer == 0)

	var collision_mask = PhysicsServer3D.body_get_collision_mask(body_rid)
	assert(collision_mask == 0)

	var collision_priority = PhysicsServer3D.body_get_collision_priority(body_rid)
	assert(collision_priority == 0.0)

	var constant_force = PhysicsServer3D.body_get_constant_force(body_rid)
	assert(constant_force == Vector3(0, 0, 0))

	var constant_torque = PhysicsServer3D.body_get_constant_torque(body_rid)
	assert(constant_torque == Vector3(0, 0, 0))

	var ccd_enabled = PhysicsServer3D.body_is_continuous_collision_detection_enabled(body_rid)
	assert(ccd_enabled == false)

	var direct_state = PhysicsServer3D.body_get_direct_state(body_rid)
	assert(direct_state == null)

	var max_contacts_reported = PhysicsServer3D.body_get_max_contacts_reported(body_rid)
	assert(max_contacts_reported <= 0)

	var body_mode = PhysicsServer3D.body_get_mode(body_rid)
	assert(body_mode == PhysicsServer3D.BODY_MODE_STATIC)

	var object_instance_id = PhysicsServer3D.body_get_object_instance_id(body_rid)
	assert(object_instance_id == 0)

	PhysicsServer3D.body_get_param(body_rid, PhysicsServer3D.BodyParameter.BODY_PARAM_ANGULAR_DAMP)

	shape_rid = PhysicsServer3D.body_get_shape(body_rid, 0)
	assert(shape_rid == RID())

	var shape_count = PhysicsServer3D.body_get_shape_count(body_rid)
	assert(shape_count <= 0)

	var shape_transform = PhysicsServer3D.body_get_shape_transform(body_rid, 0)
	assert(shape_transform.origin == Vector3())
	assert(shape_transform.basis == Basis())

	space_rid = PhysicsServer3D.body_get_space(body_rid)
	assert(space_rid == RID())

	var state_value = PhysicsServer3D.body_get_state(body_rid, PhysicsServer3D.BodyState.BODY_STATE_ANGULAR_VELOCITY)
	assert(state_value == null)

	var omit_force_integration = PhysicsServer3D.body_is_omitting_force_integration(body_rid)
	assert(omit_force_integration == false)

	PhysicsServer3D.body_remove_collision_exception(body_rid, excepted_body_rid)
	PhysicsServer3D.body_remove_shape(body_rid, 0)
	PhysicsServer3D.body_reset_mass_properties(body_rid)
	PhysicsServer3D.body_set_axis_velocity(body_rid, axis_velocity)
	PhysicsServer3D.body_set_collision_layer(body_rid, 0)
	PhysicsServer3D.body_set_collision_mask(body_rid, 0)
	PhysicsServer3D.body_set_collision_priority(body_rid, priority)
	PhysicsServer3D.body_set_constant_force(body_rid, force)
	PhysicsServer3D.body_set_constant_torque(body_rid, torque)
	PhysicsServer3D.body_set_enable_continuous_collision_detection(body_rid, false)
	PhysicsServer3D.body_set_force_integration_callback(body_rid, callback)
	PhysicsServer3D.body_set_max_contacts_reported(body_rid, 0)
	PhysicsServer3D.body_set_mode(body_rid, body_mode)
	PhysicsServer3D.body_set_omit_force_integration(body_rid, true)
	PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BodyParameter.BODY_PARAM_ANGULAR_DAMP, 1.2)
	PhysicsServer3D.body_set_shape(body_rid, 0, shape_rid)
	PhysicsServer3D.body_set_shape_disabled(body_rid, 0, true)
	PhysicsServer3D.body_set_shape_transform(body_rid, 0, new_transform)
	PhysicsServer3D.body_set_space(body_rid, space_rid)
	PhysicsServer3D.body_set_state(body_rid, PhysicsServer3D.BodyState.BODY_STATE_ANGULAR_VELOCITY, Vector3(0, 0, 2.3))

# Regression test for https://github.com/appsinacup/godot-rapier-physics/issues/550
# PhysicsDirectBodyState3D.center_of_mass must return the center of mass offset
# relative to the body origin expressed in global orientation (rotated - and scaled -
# by the body basis but NOT translated by the body origin), matching Godot's built-in
# physics server. Covered across a table of transform configurations: with/without
# rotation, with/without translation, with scale and negative values.
func test_body_center_of_mass_3d():
	print("test_body_center_of_mass_3d")

	# Each case: [name, local_com, transform, expected_global].
	# The expected global center_of_mass is the local offset rotated into global orientation,
	# WITHOUT the origin translation. Physics bodies ignore transform scale (scale lives on
	# the shapes), so a scaled transform must produce the same result as its rotation alone.
	var rot_z := Basis(Vector3(0, 0, 1), PI / 2.0)
	var rot_x := Basis(Vector3(1, 0, 0), PI / 2.0)
	var scaled_basis := Basis(Vector3(0, 0, 1), PI / 2.0).scaled(Vector3(2, 2, 2))
	var cases = [
		["rotation only", Vector3(3, 0, 0), Transform3D(rot_z, Vector3.ZERO), rot_z * Vector3(3, 0, 0)],
		["translation only", Vector3(3, 0, 0), Transform3D(Basis.IDENTITY, Vector3(100, 50, 20)), Vector3(3, 0, 0)],
		["rotation and translation", Vector3(3, 0, 0), Transform3D(rot_z, Vector3(100, 50, 20)), rot_z * Vector3(3, 0, 0)],
		["rotation about x", Vector3(0, 4, 1), Transform3D(rot_x, Vector3(-10, 5, 30)), rot_x * Vector3(0, 4, 1)],
		["uniform scale is ignored", Vector3(3, -2, 1), Transform3D(scaled_basis, Vector3(10, -30, 5)), rot_z * Vector3(3, -2, 1)],
		["zero center of mass", Vector3.ZERO, Transform3D(Basis(Vector3(0, 1, 0), PI / 3.0), Vector3(20, 20, 20)), Vector3.ZERO],
	]

	for c in cases:
		var case_name: String = c[0]
		var local_com: Vector3 = c[1]
		var xform: Transform3D = c[2]
		var expected_global: Vector3 = c[3]

		var body_rid = PhysicsServer3D.body_create()
		var space_rid = get_viewport().world_3d.space
		PhysicsServer3D.body_set_space(body_rid, space_rid)
		PhysicsServer3D.body_set_mode(body_rid, PhysicsServer3D.BODY_MODE_RIGID)
		PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BODY_PARAM_CENTER_OF_MASS, local_com)
		PhysicsServer3D.body_set_state(body_rid, PhysicsServer3D.BODY_STATE_TRANSFORM, xform)

		var direct_state = PhysicsServer3D.body_get_direct_state(body_rid)
		assert(direct_state != null, "[%s] direct state must be available" % case_name)

		# center_of_mass_local: the offset in the body's local frame.
		assert(direct_state.center_of_mass_local.is_equal_approx(local_com),
			"[%s] center_of_mass_local expected %s, got %s" % [case_name, local_com, direct_state.center_of_mass_local])

		# center_of_mass: the offset rotated into global orientation, NOT translated.
		assert(direct_state.center_of_mass.is_equal_approx(expected_global),
			"[%s] center_of_mass expected %s, got %s" % [case_name, expected_global, direct_state.center_of_mass])

		# When the origin is non-zero and the offset non-zero, center_of_mass must NOT be
		# the absolute global position of the COM (which the pre-fix code returned). The two
		# differ exactly by the origin, so only assert it when the origin is non-zero.
		if xform.origin != Vector3.ZERO and local_com != Vector3.ZERO:
			assert(!direct_state.center_of_mass.is_equal_approx(xform * local_com),
				"[%s] center_of_mass must not include the origin translation" % case_name)

		PhysicsServer3D.free_rid(body_rid)

func test_body():
	print("test_body")

	var body_rid = PhysicsServer3D.body_create()
	var space_rid = get_viewport().world_3d.space
	var shape_rid = RID()
	PhysicsServer3D.body_set_space(body_rid, space_rid)

	var new_transform = Transform3D()
	var force = Vector3(1, 2, 3)
	var impulse = Vector3(1, 2, 3)
	var callback = Callable()
	var axis_velocity = Vector3(1, 2, 3)
	var torque = Vector3(0, 0, 10)

	PhysicsServer3D.body_add_constant_central_force(body_rid, force)
	PhysicsServer3D.body_add_constant_force(body_rid, force)
	PhysicsServer3D.body_add_constant_torque(body_rid, torque)
	PhysicsServer3D.body_add_shape(body_rid, shape_rid, new_transform)
	PhysicsServer3D.body_apply_central_force(body_rid, force)
	PhysicsServer3D.body_apply_central_impulse(body_rid, impulse)
	PhysicsServer3D.body_apply_force(body_rid, force, impulse)
	PhysicsServer3D.body_apply_impulse(body_rid, impulse, force)
	PhysicsServer3D.body_apply_torque(body_rid, torque)
	PhysicsServer3D.body_apply_torque_impulse(body_rid, torque)
	PhysicsServer3D.body_attach_object_instance_id(body_rid, 2)
	PhysicsServer3D.body_clear_shapes(body_rid)


	PhysicsServer3D.body_remove_shape(body_rid, 0)
	PhysicsServer3D.body_reset_mass_properties(body_rid)
	PhysicsServer3D.body_set_axis_velocity(body_rid, axis_velocity)
	PhysicsServer3D.body_set_collision_layer(body_rid, 1)
	PhysicsServer3D.body_set_collision_mask(body_rid, 2)
	PhysicsServer3D.body_set_collision_priority(body_rid, 1.0)
	PhysicsServer3D.body_set_constant_force(body_rid, force)
	PhysicsServer3D.body_set_constant_torque(body_rid, torque)
	PhysicsServer3D.body_set_enable_continuous_collision_detection(body_rid, false)
	PhysicsServer3D.body_set_force_integration_callback(body_rid, callback)
	PhysicsServer3D.body_set_max_contacts_reported(body_rid, 0)
	PhysicsServer3D.body_set_mode(body_rid, PhysicsServer3D.BODY_MODE_RIGID)
	PhysicsServer3D.body_set_omit_force_integration(body_rid, true)
	PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BodyParameter.BODY_PARAM_ANGULAR_DAMP, 3)
	PhysicsServer3D.body_set_shape(body_rid, 0, shape_rid)
	PhysicsServer3D.body_set_shape_disabled(body_rid, 0, true)
	PhysicsServer3D.body_set_shape_transform(body_rid, 0, new_transform)
	PhysicsServer3D.body_set_state(body_rid, PhysicsServer3D.BodyState.BODY_STATE_ANGULAR_VELOCITY, Vector3(0, 0, 4.0))

	var collision_layer = PhysicsServer3D.body_get_collision_layer(body_rid)
	assert(collision_layer == 1)

	var collision_mask = PhysicsServer3D.body_get_collision_mask(body_rid)
	assert(collision_mask == 2)

	var constant_force = PhysicsServer3D.body_get_constant_force(body_rid)
	assert(constant_force == Vector3(1, 2, 3))

	var constant_torque = PhysicsServer3D.body_get_constant_torque(body_rid)
	assert(constant_torque == Vector3(0, 0, 10))

	var ccd_enabled = PhysicsServer3D.body_is_continuous_collision_detection_enabled(body_rid)
	assert(ccd_enabled == false)

	var direct_state = PhysicsServer3D.body_get_direct_state(body_rid)
	assert(direct_state != null)

	var max_contacts_reported = PhysicsServer3D.body_get_max_contacts_reported(body_rid)
	assert(max_contacts_reported <= 0)

	var body_mode = PhysicsServer3D.body_get_mode(body_rid)
	assert(body_mode == PhysicsServer3D.BODY_MODE_RIGID)

	var object_instance_id = PhysicsServer3D.body_get_object_instance_id(body_rid)
	assert(object_instance_id == 2)

	var param_value = PhysicsServer3D.body_get_param(body_rid, PhysicsServer3D.BodyParameter.BODY_PARAM_ANGULAR_DAMP)
	assert(param_value == 3)

	shape_rid = PhysicsServer3D.body_get_shape(body_rid, 0)
	assert(shape_rid == RID())

	var shape_count = PhysicsServer3D.body_get_shape_count(body_rid)
	assert(shape_count <= 0)

	var shape_transform := PhysicsServer3D.body_get_shape_transform(body_rid, 0)
	assert(shape_transform.origin == Vector3())
	assert(shape_transform.basis == Basis())

	space_rid = PhysicsServer3D.body_get_space(body_rid)
	assert(space_rid != RID())

	var state_value = PhysicsServer3D.body_get_state(body_rid, PhysicsServer3D.BodyState.BODY_STATE_ANGULAR_VELOCITY)
	assert(state_value == Vector3(0, 0, 4))

	var omit_force_integration = PhysicsServer3D.body_is_omitting_force_integration(body_rid)
	assert(omit_force_integration == true)


	PhysicsServer3D.free_rid(body_rid)
