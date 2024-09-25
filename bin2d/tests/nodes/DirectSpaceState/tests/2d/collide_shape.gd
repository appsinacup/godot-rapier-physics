extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks all the [PhysicsShapeQueryParameters2D] of [collide_shape]
	"""
	
func test_name() -> String:
	return "DirectSpaceState2D | testing [collide_shape] from [get_world_2d().direct_space_state]"

var result = []
func test_start() -> void:

	# Add Area on the LEFT
	var area := add_area(CENTER_LEFT)
	
	# Add Body on the RIGHT
	var body := add_body(CENTER_RIGHT - Vector2(100, 0))
	body.set_collision_layer_value(2, true)
	body.set_collision_mask_value(2, true)
	var body2 := add_body(BOTTOM_RIGHT)
	
	var mid_screen_width := Global.WINDOW_SIZE.x/2
	
	var d_space := get_world_2d().direct_space_state
	var shape_rid = PhysicsServer2D.rectangle_shape_create()
	var size = Vector2(20,20)
	var shape_type = PhysicsServer2D.shape_get_type(shape_rid)
	PhysicsServer2D.shape_set_data(shape_rid, size)

	var checks_point = func(p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2: # avoid a bug in first frame
			return
		
		if true: # limit the scope
			p_monitor.add_test("Can collide with Body")
			var body_query := PhysicsShapeQueryParameters2D.new()
			body_query.collide_with_bodies = true
			body_query.shape_rid = shape_rid
			body_query.transform = Transform2D(0, CENTER)
			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
			result = d_space.collide_shape(body_query)
			queue_redraw()
			
#		if true: # limit the scope
#			p_monitor.add_test("Return [1,1] when shape is inside a Body")
#			var body_query := PhysicsShapeQueryParameters2D.new()
#			body_query.collide_with_bodies = true
#			body_query.shape_rid = shape_rid
#			body_query.transform = Transform2D(0, CENTER_RIGHT)
#			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
#			var result = d_space.cast_motion(body_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		if true: # limit the scope
#			p_monitor.add_test("Can not collide with Body")
#			var body_query := PhysicsShapeQueryParameters2D.new()
#			body_query.collide_with_bodies = false
#			body_query.shape_rid = shape_rid
#			body_query.transform = Transform2D(0, CENTER)
#			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
#			var result = d_space.cast_motion(body_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		if true:
#			p_monitor.add_test("Can collide with Area")
#			var area_query := PhysicsShapeQueryParameters2D.new()
#			area_query.shape_rid = shape_rid
#			area_query.transform = Transform2D(0, CENTER)
#			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
#			area_query.collide_with_areas = true
#			var result = d_space.cast_motion(area_query)
#			p_monitor.add_test_result(!result.is_empty() and is_between(result, 0.0126, 0.016))
#
#		if true:
#			p_monitor.add_test("Can not collide with Area")
#			var area_query := PhysicsShapeQueryParameters2D.new()
#			area_query.shape_rid = shape_rid
#			area_query.transform = Transform2D(0, CENTER)
#			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
#			area_query.collide_with_areas = false
#			var result = d_space.cast_motion(area_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		# Can exclude a RID
#		if true:
#			p_monitor.add_test("Can exclude a Body by RID")
#			var exclude_rid_query := PhysicsShapeQueryParameters2D.new()
#			exclude_rid_query.collide_with_bodies = true
#			exclude_rid_query.transform = Transform2D(0, CENTER)
#			exclude_rid_query.shape_rid = shape_rid
#			exclude_rid_query.motion = Vector2(mid_screen_width / 0.016, 0)
#			exclude_rid_query.exclude = [body.get_rid()]
#			var result = d_space.cast_motion(exclude_rid_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		if true:
#			p_monitor.add_test("Can exclude an Area by RID")
#			var area_query := PhysicsShapeQueryParameters2D.new()
#			area_query.shape_rid = shape_rid
#			area_query.transform = Transform2D(0, CENTER)
#			area_query.collide_with_areas = true
#			area_query.exclude = [area.get_rid()]
#			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
#			var result = d_space.cast_motion(area_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		if true:
#			p_monitor.add_test("Can exclude multiple RID")
#			var exclude_rid_query := PhysicsShapeQueryParameters2D.new()
#			exclude_rid_query.transform = Transform2D(0, TOP_RIGHT)
#			exclude_rid_query.collide_with_bodies = true
#			exclude_rid_query.shape_rid = shape_rid
#			exclude_rid_query.motion = Vector2(0 ,mid_screen_width / 0.016)
#			exclude_rid_query.exclude = [body.get_rid(), body2.get_rid()]
#			var result = d_space.cast_motion(exclude_rid_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		if true:
#			p_monitor.add_test("Don't report collision in the wrong collision layer")
#			var area_query := PhysicsShapeQueryParameters2D.new()
#			area_query.shape_rid = shape_rid
#			area_query.transform = Transform2D(0, CENTER)
#			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
#			area_query.collide_with_areas = true
#			area_query.collision_mask = pow(2, 2-1) # second layer
#			var result = d_space.cast_motion(area_query)
#			p_monitor.add_test_result(!result.is_empty() and is_eq(result, [1,1]))
#
#		if true:
#			p_monitor.add_test("Report collision in good collision layer")
#			var body_query := PhysicsShapeQueryParameters2D.new()
#			body_query.shape_rid = shape_rid
#			body_query.transform = Transform2D(0, CENTER)
#			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
#			body_query.collide_with_bodies = true
#			body_query.collision_mask = pow(2, 2-1) # second layer
#			var result = d_space.cast_motion(body_query)
#			p_monitor.add_test_result(!result.is_empty() and is_between(result, 0.0126, 0.016))
#
#		# Rotation
#		if true:
#			p_monitor.add_test("Can apply rotation to the shape")
#			# Without Rotation Don't collide
#			var body_query := PhysicsShapeQueryParameters2D.new()
#			body_query.shape_rid = shape_rid
#			body_query.transform = Transform2D(0, Vector2(CENTER.x, 261-10))
#			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
#			body_query.collide_with_bodies = true
#			var result1 = d_space.cast_motion(body_query)
#			# With rotation collide
#			var body_query_rot := PhysicsShapeQueryParameters2D.new()
#			body_query_rot.shape_rid = shape_rid
#			body_query_rot.transform = Transform2D(deg_to_rad(45), Vector2(CENTER.x, 261-10))
#			body_query_rot.motion = Vector2(mid_screen_width / 0.016, 0)
#			body_query_rot.collide_with_bodies = true
#			var result2 = d_space.cast_motion(body_query_rot)
#			p_monitor.add_test_result(!result1.is_empty() and !result2.is_empty() and is_eq(result1, [1,1]) and is_between(result2, 0.0126, 0.016))
		
		PhysicsServer2D.free_rid(shape_rid)
		#p_monitor.monitor_completed()

	var check_max_stability_monitor := create_generic_manual_monitor(self, checks_point, simulation_duration)

func add_body(p_position: Vector2, p_add_child := true) -> StaticBody2D:
	var body := StaticBody2D.new()
	var body_shape := get_default_collision_shape(PhysicsTest2D.TestCollisionShape.RECTANGLE, 4)
	body.add_child(body_shape)
	body.position = p_position
	if p_add_child:
		add_child(body)
	return body
	
func add_area(p_position: Vector2, p_add_child := true) -> Area2D:
	var area := Area2D.new()
	area.add_child(get_default_collision_shape(PhysicsTest2D.TestCollisionShape.RECTANGLE, 4))
	area.position = p_position
	if p_add_child:
		add_child(area)
	return area

func _process(delta: float) -> void:
	queue_redraw()
func _draw() -> void:
	for col in result:
		draw_circle(col, 1, Color.RED)
