extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks the basic API for [Area2D]
	"""
	
func test_name() -> String:
	return "Area2D | testing the basic API"
	
func test_start() -> void:
	# Add Area in the center
	var area := add_area(CENTER)
	area.monitoring = true
	area.monitorable = false
	
	# Add rigid body
	var rigid_body := add_rigid_body(CENTER)
	
	# Prepare added rigid body for later
	var rigid_body_add := add_rigid_body(CENTER)
	rigid_body_add.disable_mode = CollisionObject2D.DISABLE_MODE_REMOVE
	rigid_body_add.process_mode = Node.PROCESS_MODE_DISABLED
	
	# Add rigid body on different collision layer
	var rigid_body2 := add_rigid_body(CENTER)
	rigid_body2.set_collision_layer_value(1, false)
	rigid_body2.set_collision_layer_value(2, true)
	rigid_body2.set_collision_mask_value(1, false)
	rigid_body2.set_collision_mask_value(2, true)
	
	# Add rigid body with multiple shapes
	var area_bottom_center := add_area(BOTTOM_CENTER)
	var rigid_body_composite := add_rigid_body(BOTTOM_CENTER)
	var body_shape2 := PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.CIRCLE, 2)
	rigid_body_composite.add_child(body_shape2)
	
	# Add area and kinematic body in the top left
	var area_top_left := add_area(TOP_LEFT)
	var kinematic_body := add_rigid_body(TOP_LEFT)
	kinematic_body.freeze_mode = RigidBody2D.FREEZE_MODE_KINEMATIC
	kinematic_body.freeze = true
	
	# Add area and static body in the top right
	var area_top_right := add_area(TOP_RIGHT)
	var static_body := add_static_body(TOP_RIGHT)
	
	# Add Area to be detected but with detection disabled
	var area_detected := add_area(CENTER)
	area_detected.monitoring = false
	area_detected.monitorable = true
	
	# Add Area not to be detected and with detection disabled
	var area_inactive := add_area(CENTER)
	area_inactive.monitoring = false
	area_inactive.monitorable = false
	
	# Add Area in the center left
	var area_center_left := add_area(CENTER_LEFT)
	area_center_left.monitoring = true
	area_center_left.monitorable = false
	
	var checks_point = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 2:
			if true: # limit the scope
				p_monitor.add_test("Detect rigid body")
				var result := area.overlaps_body(rigid_body)
				p_monitor.add_test_result(result)

			if true: # limit the scope
				p_monitor.add_test("Don't detect disabled rigid body")
				var result := area.overlaps_body(rigid_body_add)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Detect rigid body with multiple shapes")
				var result := area_bottom_center.overlaps_body(rigid_body_composite)
				p_monitor.add_test_result(result)

			if true:
				p_monitor.add_test("Detect kinematic body")
				var result := area_top_left.overlaps_body(kinematic_body)
				p_monitor.add_test_result(result)

			if true:
				p_monitor.add_test("Detect static body")
				var result := area_top_right.overlaps_body(static_body)
				p_monitor.add_test_result(result)

			if true:
				p_monitor.add_test("Don't detect rigid body on collision layer 2")
				var result := area.overlaps_body(rigid_body2)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Detect monitorable area")
				var result := area.overlaps_area(area_detected)
				p_monitor.add_test_result(result)

			if true:
				p_monitor.add_test("Don't detect unmonitorable area")
				var result := area.overlaps_area(area_inactive)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Don't detect rigid body when not monitoring")
				var result := area_inactive.overlaps_body(rigid_body)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Don't detect monitorable area when not monitoring")
				var result := area_inactive.overlaps_body(area_detected)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Don't detect anything when not overlapping")
				var result_bodies := area_center_left.has_overlapping_bodies()
				var result_areas := area_center_left.has_overlapping_areas()
				if result_bodies:
					p_monitor.add_test_error("Body detected.")
				if result_areas:
					p_monitor.add_test_error("Area detected.")
				p_monitor.add_test_result(!result_bodies && !result_areas)
		
			# Add rigid body
			rigid_body_add.process_mode = Node.PROCESS_MODE_INHERIT
			
			# Remove rigid body
			rigid_body.disable_mode = CollisionObject2D.DISABLE_MODE_REMOVE
			rigid_body.process_mode = Node.PROCESS_MODE_DISABLED

			# Disable a shape from the composite rigid body
			body_shape2.disabled = true
		
		if p_monitor.frame == 3: # wait for one frame
			# Tests after modifications
			
			if true:
				p_monitor.add_test("Detect added rigid body")
				var result := area.overlaps_body(rigid_body_add)
				p_monitor.add_test_result(result)
			
			if true:
				p_monitor.add_test("Don't detect removed rigid body")
				var result := area.overlaps_body(rigid_body)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Detect rigid body with one shape disabled")
				var result := area_bottom_center.overlaps_body(rigid_body_composite)
				p_monitor.add_test_result(result)
			
			# Disable remaining shape for composite rigid body
			rigid_body_composite.get_child(0).disabled = true
			
			# Move added rigid body
			var new_transform = Transform2D(0.0, CENTER_LEFT)
			PhysicsServer2D.body_set_state(rigid_body_add.get_rid(), PhysicsServer2D.BODY_STATE_TRANSFORM, new_transform)
			
			# Re-add rigid body
			rigid_body.process_mode = Node.PROCESS_MODE_INHERIT
			
		if p_monitor.frame == 4: # wait for one frame
			# Tests after modifications
			
			if true:
				p_monitor.add_test("Don't detect rigid body with all shapes disabled")
				var result := area_bottom_center.overlaps_body(rigid_body_composite)
				p_monitor.add_test_result(!result)
			
			if true:
				p_monitor.add_test("Don't detect rigid body moved outside area")
				var result := area.overlaps_body(rigid_body_add)
				p_monitor.add_test_result(!result)

			if true:
				p_monitor.add_test("Detect rigid body moved inside area")
				var result := area_center_left.overlaps_body(rigid_body_add)
				p_monitor.add_test_result(result)

			# Disable area
			area.disable_mode = CollisionObject2D.DISABLE_MODE_REMOVE
			area.process_mode = Node.PROCESS_MODE_DISABLED
			
		if p_monitor.frame == 5: # wait for one frame
			# Tests after modifications

			if true:
				p_monitor.add_test("Don't detect anything for disabled area")
				var result_bodies := area.has_overlapping_bodies()
				var result_areas := area.has_overlapping_areas()
				if result_bodies:
					p_monitor.add_test_error("Body detected.")
				if result_areas:
					p_monitor.add_test_error("Area detected.")
				p_monitor.add_test_result(!result_bodies && !result_areas)

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
	body.gravity_scale = 0.0
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
