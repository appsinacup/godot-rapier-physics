extends Node2D

func _ready():
	test_body_create()
	test_body_empty()
	test_body()
	test_body_collision_exception_invalid()
	print("Body tests passed")

func test_body_collision_exception_invalid():
	PhysicsServer2D.body_add_collision_exception(RID(), RID())
	PhysicsServer2D.body_remove_collision_exception(RID(), RID())

func test_body_create():
	var body = PhysicsServer2D.body_create()
	var space = get_viewport().world_2d.space
	assert(body.is_valid())
	assert(!PhysicsServer2D.body_get_space(body).is_valid())
	PhysicsServer2D.body_set_space(body, RID())
	assert(!PhysicsServer2D.body_get_space(body).is_valid())
	PhysicsServer2D.body_set_space(body, space)
	assert(PhysicsServer2D.body_get_space(body) == space)
	PhysicsServer2D.body_set_space(body, RID())
	assert(!PhysicsServer2D.body_get_space(body).is_valid())
	PhysicsServer2D.body_set_space(body, space)
	PhysicsServer2D.free_rid(body)
	assert(!PhysicsServer2D.body_get_space(body).is_valid())
	PhysicsServer2D.free_rid(body)
	PhysicsServer2D.free_rid(RID())
	
func test_body_empty():
	var body_rid = RID()
	
	var shape_rid = RID()
	var transform = Transform2D()
	var force = Vector2(0, 0)
	var impulse = Vector2(0, 0)
	var callback = Callable()
	var excepted_body_rid = RID()
	var space_rid = RID()
	var axis_velocity = Vector2(1, 2)
	var priority = 1.0
	var torque = 10.0
	
	PhysicsServer2D.body_add_constant_central_force(body_rid, force)
	PhysicsServer2D.body_add_constant_force(body_rid, force)
	PhysicsServer2D.body_add_constant_torque(body_rid, torque)
	PhysicsServer2D.body_add_shape(body_rid, shape_rid, transform)
	PhysicsServer2D.body_apply_central_force(body_rid, force)
	PhysicsServer2D.body_apply_central_impulse(body_rid, impulse)
	PhysicsServer2D.body_apply_force(body_rid, force, impulse)
	PhysicsServer2D.body_apply_impulse(body_rid, impulse, force)
	PhysicsServer2D.body_apply_torque(body_rid, torque)
	PhysicsServer2D.body_apply_torque_impulse(body_rid, torque)
	PhysicsServer2D.body_attach_canvas_instance_id(body_rid, 0)
	PhysicsServer2D.body_attach_object_instance_id(body_rid, 0)
	PhysicsServer2D.body_clear_shapes(body_rid)

	var canvas_instance_id = PhysicsServer2D.body_get_canvas_instance_id(body_rid)
	assert(canvas_instance_id == 0)

	var collision_layer = PhysicsServer2D.body_get_collision_layer(body_rid)
	assert(collision_layer == 0)

	var collision_mask = PhysicsServer2D.body_get_collision_mask(body_rid)
	assert(collision_mask == 0)

	var collision_priority = PhysicsServer2D.body_get_collision_priority(body_rid)
	assert(collision_priority == 0.0)

	var constant_force = PhysicsServer2D.body_get_constant_force(body_rid)
	assert(constant_force == Vector2(0, 0))

	var constant_torque = PhysicsServer2D.body_get_constant_torque(body_rid)
	assert(constant_torque == 0.0)

	var ccd_mode = PhysicsServer2D.body_get_continuous_collision_detection_mode(body_rid)
	assert(ccd_mode == PhysicsServer2D.CCD_MODE_DISABLED)

	var direct_state = PhysicsServer2D.body_get_direct_state(body_rid)
	assert(direct_state == null)

	var max_contacts_reported = PhysicsServer2D.body_get_max_contacts_reported(body_rid)
	assert(max_contacts_reported <= 0)

	var body_mode = PhysicsServer2D.body_get_mode(body_rid)
	assert(body_mode == PhysicsServer2D.BODY_MODE_STATIC)

	var object_instance_id = PhysicsServer2D.body_get_object_instance_id(body_rid)
	assert(object_instance_id == 0)

	var param_value = PhysicsServer2D.body_get_param(body_rid, PhysicsServer2D.BodyParameter.BODY_PARAM_ANGULAR_DAMP)
	#assert(param_value == null)

	shape_rid = PhysicsServer2D.body_get_shape(body_rid, 0)
	assert(shape_rid == RID())

	var shape_count = PhysicsServer2D.body_get_shape_count(body_rid)
	assert(shape_count <= 0)

	var shape_transform = PhysicsServer2D.body_get_shape_transform(body_rid, 0)
	assert(shape_transform.origin == Vector2())
	assert(shape_transform.get_rotation() == 0.0)

	space_rid = PhysicsServer2D.body_get_space(body_rid)
	assert(space_rid == RID())

	var state_value = PhysicsServer2D.body_get_state(body_rid, PhysicsServer2D.BodyState.BODY_STATE_ANGULAR_VELOCITY)
	assert(state_value == null)

	var omit_force_integration = PhysicsServer2D.body_is_omitting_force_integration(body_rid)
	assert(omit_force_integration == false)

	PhysicsServer2D.body_remove_collision_exception(body_rid, excepted_body_rid)
	PhysicsServer2D.body_remove_shape(body_rid, 0)
	PhysicsServer2D.body_reset_mass_properties(body_rid)
	PhysicsServer2D.body_set_axis_velocity(body_rid, axis_velocity)
	PhysicsServer2D.body_set_collision_layer(body_rid, 0)
	PhysicsServer2D.body_set_collision_mask(body_rid, 0)
	PhysicsServer2D.body_set_collision_priority(body_rid, priority)
	PhysicsServer2D.body_set_constant_force(body_rid, force)
	PhysicsServer2D.body_set_constant_torque(body_rid, torque)
	PhysicsServer2D.body_set_continuous_collision_detection_mode(body_rid, ccd_mode)
	PhysicsServer2D.body_set_force_integration_callback(body_rid, callback)
	PhysicsServer2D.body_set_max_contacts_reported(body_rid, 0)
	PhysicsServer2D.body_set_mode(body_rid, body_mode)
	PhysicsServer2D.body_set_omit_force_integration(body_rid, true)
	PhysicsServer2D.body_set_param(body_rid, PhysicsServer2D.BodyParameter.BODY_PARAM_ANGULAR_DAMP, 1.2)
	PhysicsServer2D.body_set_shape(body_rid, 0, shape_rid)
	PhysicsServer2D.body_set_shape_as_one_way_collision(body_rid, 0, true, 0.0)
	PhysicsServer2D.body_set_shape_disabled(body_rid, 0, true)
	PhysicsServer2D.body_set_shape_transform(body_rid, 0, transform)
	PhysicsServer2D.body_set_space(body_rid, space_rid)
	PhysicsServer2D.body_set_state(body_rid, PhysicsServer2D.BodyState.BODY_STATE_ANGULAR_VELOCITY, 2.3)
	print("Body functions test passed")

func test_body():
	var body_rid = PhysicsServer2D.body_create()
	var space_rid = get_viewport().world_2d.space
	var shape_rid = RID()
	PhysicsServer2D.body_set_space(body_rid, space_rid)
	
	var transform = Transform2D()
	var force = Vector2(1, 2)
	var impulse = Vector2(1, 2)
	var callback = Callable()
	var excepted_body_rid = RID()
	var axis_velocity = Vector2(1, 2)
	var torque = 10.0
	
	PhysicsServer2D.body_add_constant_central_force(body_rid, force)
	PhysicsServer2D.body_add_constant_force(body_rid, force)
	PhysicsServer2D.body_add_constant_torque(body_rid, torque)
	PhysicsServer2D.body_add_shape(body_rid, shape_rid, transform)
	PhysicsServer2D.body_apply_central_force(body_rid, force)
	PhysicsServer2D.body_apply_central_impulse(body_rid, impulse)
	PhysicsServer2D.body_apply_force(body_rid, force, impulse)
	PhysicsServer2D.body_apply_impulse(body_rid, impulse, force)
	PhysicsServer2D.body_apply_torque(body_rid, torque)
	PhysicsServer2D.body_apply_torque_impulse(body_rid, torque)
	PhysicsServer2D.body_attach_canvas_instance_id(body_rid, 1)
	PhysicsServer2D.body_attach_object_instance_id(body_rid, 2)
	PhysicsServer2D.body_clear_shapes(body_rid)


	PhysicsServer2D.body_remove_shape(body_rid, 0)
	PhysicsServer2D.body_reset_mass_properties(body_rid)
	PhysicsServer2D.body_set_axis_velocity(body_rid, axis_velocity)
	PhysicsServer2D.body_set_collision_layer(body_rid, 1)
	PhysicsServer2D.body_set_collision_mask(body_rid, 2)
	PhysicsServer2D.body_set_collision_priority(body_rid, 1.0)
	PhysicsServer2D.body_set_constant_force(body_rid, force)
	PhysicsServer2D.body_set_constant_torque(body_rid, torque)
	PhysicsServer2D.body_set_continuous_collision_detection_mode(body_rid, PhysicsServer2D.CCD_MODE_DISABLED)
	PhysicsServer2D.body_set_force_integration_callback(body_rid, callback)
	PhysicsServer2D.body_set_max_contacts_reported(body_rid, 0)
	PhysicsServer2D.body_set_mode(body_rid, PhysicsServer2D.BODY_MODE_RIGID)
	PhysicsServer2D.body_set_omit_force_integration(body_rid, true)
	PhysicsServer2D.body_set_param(body_rid, PhysicsServer2D.BodyParameter.BODY_PARAM_ANGULAR_DAMP, 3)
	PhysicsServer2D.body_set_shape(body_rid, 0, shape_rid)
	PhysicsServer2D.body_set_shape_as_one_way_collision(body_rid, 0, true, 0.0)
	PhysicsServer2D.body_set_shape_disabled(body_rid, 0, true)
	PhysicsServer2D.body_set_shape_transform(body_rid, 0, transform)
	PhysicsServer2D.body_set_state(body_rid, PhysicsServer2D.BodyState.BODY_STATE_ANGULAR_VELOCITY, 4.0)

	var canvas_instance_id = PhysicsServer2D.body_get_canvas_instance_id(body_rid)
	assert(canvas_instance_id == 1)

	var collision_layer = PhysicsServer2D.body_get_collision_layer(body_rid)
	assert(collision_layer == 1)

	var collision_mask = PhysicsServer2D.body_get_collision_mask(body_rid)
	assert(collision_mask == 2)

	var constant_force = PhysicsServer2D.body_get_constant_force(body_rid)
	assert(constant_force == Vector2(1, 2))

	var constant_torque = PhysicsServer2D.body_get_constant_torque(body_rid)
	assert(constant_torque == 10.0)

	var ccd_mode = PhysicsServer2D.body_get_continuous_collision_detection_mode(body_rid)
	assert(ccd_mode == PhysicsServer2D.CCD_MODE_DISABLED)

	var direct_state = PhysicsServer2D.body_get_direct_state(body_rid)
	assert(direct_state != null)

	var max_contacts_reported = PhysicsServer2D.body_get_max_contacts_reported(body_rid)
	assert(max_contacts_reported <= 0)

	var body_mode = PhysicsServer2D.body_get_mode(body_rid)
	assert(body_mode == PhysicsServer2D.BODY_MODE_RIGID)

	var object_instance_id = PhysicsServer2D.body_get_object_instance_id(body_rid)
	assert(object_instance_id == 2)

	var param_value = PhysicsServer2D.body_get_param(body_rid, PhysicsServer2D.BodyParameter.BODY_PARAM_ANGULAR_DAMP)
	assert(param_value == 3)

	shape_rid = PhysicsServer2D.body_get_shape(body_rid, 0)
	assert(shape_rid == RID())

	var shape_count = PhysicsServer2D.body_get_shape_count(body_rid)
	assert(shape_count <= 0)

	var shape_transform := PhysicsServer2D.body_get_shape_transform(body_rid, 0)
	assert(shape_transform.origin == Vector2())
	assert(shape_transform.get_rotation() == 0.0)

	space_rid = PhysicsServer2D.body_get_space(body_rid)
	assert(space_rid != RID())

	var state_value = PhysicsServer2D.body_get_state(body_rid, PhysicsServer2D.BodyState.BODY_STATE_ANGULAR_VELOCITY)
	assert(state_value == 4)

	var omit_force_integration = PhysicsServer2D.body_is_omitting_force_integration(body_rid)
	assert(omit_force_integration == true)

	print("Body functions test passed")


# Test functions for space functions
func test_space_functions():
	var space_rid = RID()
	
	var created_space_rid = PhysicsServer2D.space_create()
	assert(created_space_rid != null)

	var direct_state = PhysicsServer2D.space_get_direct_state(space_rid)
	assert(direct_state == null)

	var space_param = PhysicsServer2D.space_get_param(space_rid, PhysicsServer2D.SpaceParameter.SPACE_PARAM_SOLVER_ITERATIONS)
	assert(space_param == 0.0)

	var is_active = PhysicsServer2D.space_is_active(space_rid)
	assert(is_active == false)

	PhysicsServer2D.space_set_active(space_rid, true)
	PhysicsServer2D.space_set_param(space_rid, PhysicsServer2D.SpaceParameter.SPACE_PARAM_SOLVER_ITERATIONS, 0.0)
	print("Space functions test passed")
