extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks space override for [Area2D]
	"""
	
func test_name() -> String:
	return "Area2D | testing space override"
	
func test_start() -> void:
	# Add Area with disabled gravity
	var area_no_gravity := add_area(CENTER)
	area_no_gravity.monitoring = true
	area_no_gravity.monitorable = false
	area_no_gravity.gravity_space_override = Area2D.SPACE_OVERRIDE_REPLACE
	area_no_gravity.gravity = 0.0
	
	# Add Area with custom gravity
	var area_custom_gravity := add_area(TOP_CENTER)
	area_custom_gravity.monitoring = true
	area_custom_gravity.monitorable = false
	area_custom_gravity.gravity_space_override = Area2D.SPACE_OVERRIDE_REPLACE
	area_custom_gravity.gravity_direction = Vector2.LEFT
	area_custom_gravity.gravity = 500.0
	
	# Add rigid body
	var rigid_body := add_rigid_body(CENTER)
	rigid_body.linear_damp_mode = RigidBody2D.DAMP_MODE_REPLACE
	rigid_body.linear_damp = 0.0
	rigid_body.gravity_scale = 0.0
	
	# Add rigid body
	var rigid_body2 := add_rigid_body(TOP_CENTER)
	rigid_body2.linear_damp_mode = RigidBody2D.DAMP_MODE_REPLACE
	rigid_body2.linear_damp = 0.0
	rigid_body2.gravity_scale = 0.0
	
	# Add rigid body outside area
	var rigid_body_outside := add_rigid_body(CENTER_RIGHT)
	rigid_body_outside.linear_damp_mode = RigidBody2D.DAMP_MODE_REPLACE
	rigid_body_outside.linear_damp = 0.0
	rigid_body_outside.gravity_scale = 0.0
	
	var dt := 1.0/60.0
	var default_gravity : Vector2 = ProjectSettings.get_setting("physics/2d/default_gravity_vector")
	default_gravity *= ProjectSettings.get_setting("physics/2d/default_gravity")
	
	var custom_gravity := area_custom_gravity.gravity_direction
	custom_gravity *= area_custom_gravity.gravity
	
	var checks_point = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		rigid_body.sleeping = false
		rigid_body_outside.sleeping = false
		
		if p_monitor.frame == 2:
			rigid_body.gravity_scale = 1.0
			rigid_body2.gravity_scale = 1.0
			rigid_body_outside.gravity_scale = 1.0
			
			p_monitor.data["rigid_body_pos"] = rigid_body.position
			p_monitor.data["rigid_body2_pos"] = rigid_body2.position
			p_monitor.data["rigid_body_outside_pos"] = rigid_body_outside.position
		
		if p_monitor.frame == 22:
			if true: # limit the scope
				p_monitor.add_test("Rigid body in area with no gravity doesn't move")
				var pos_start = p_monitor.data["rigid_body_pos"]
				var motion = rigid_body.position - pos_start
				var expected = Vector2.ZERO
				var success := Utils.vec2_equals(motion, expected, 0.001)
				if not success:
					p_monitor.add_test_error("Disabled gravity was not applied correctly in area, expected motion %v, got %v" % [expected, motion])
				p_monitor.add_test_result(success)
			
			if true: # limit the scope
				p_monitor.add_test("Rigid body in area with custom gravity moves to the left")
				var pos_start = p_monitor.data["rigid_body2_pos"]
				var motion = rigid_body2.position - pos_start
				var time := 20.0 * dt
				var expected = 0.5 * custom_gravity * time * time
				var success := Utils.vec2_equals(motion, expected, 4)
				if not success:
					p_monitor.add_test_error("Custom gravity was not applied correctly in area, expected motion %v, got %v" % [expected, motion])
				p_monitor.add_test_result(success)
			
			if true: # limit the scope
				p_monitor.add_test("Rigid body outside of area has default gravity applied")
				var pos_start = p_monitor.data["rigid_body_outside_pos"]
				var motion = rigid_body_outside.position - pos_start
				var time := 20.0 * dt
				var expected = 0.5 * default_gravity * time * time
				var success := Utils.vec2_equals(motion, expected, 4.0)
				if not success:
					p_monitor.add_test_error("Default gravity was not applied correctly outside of area, expected motion %v, got %v" % [expected, motion])
				p_monitor.add_test_result(success)
		
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_point, simulation_duration)

func add_static_body(p_position: Vector2, p_add_child := true) -> StaticBody2D:
	var body := StaticBody2D.new()
	var body_shape := PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.CIRCLE, 2)
	body.add_child(body_shape)
	body.position = p_position
	if p_add_child:
		add_child(body)
	return body

func add_rigid_body(p_position: Vector2, p_add_child := true) -> RigidBody2D:
	var body := RigidBody2D.new()
	var body_shape := PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.CIRCLE, 2)
	body.add_child(body_shape)
	body.position = p_position
	if p_add_child:
		add_child(body)
	return body
	
func add_area(p_position: Vector2, p_add_child := true) -> Area2D:
	var area := Area2D.new()
	area.add_child(PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.RECTANGLE, 4))
	area.position = p_position
	if p_add_child:
		add_child(area)
	return area

