extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks all the [PhysicsShapeQueryParameters2D] of [intersect_shape]
	"""
	
func test_name() -> String:
	return "DirectSpaceState2D | testing [intersect_shape] from [get_world_2d().direct_space_state]"
	
func test_start() -> void:

	var mid_screen_width := Global.WINDOW_SIZE.x/2

	# Add Area on the LEFT
	var area := add_area(CENTER_LEFT)
	
	# Add Body on the RIGHT
	var body := add_body(CENTER_RIGHT)
	body.set_collision_layer_value(2, true)
	body.set_collision_mask_value(2, true)
	var body2 := add_body(BOTTOM_RIGHT)
	
	var d_space := get_world_2d().direct_space_state
	var shape_rid = PhysicsServer2D.rectangle_shape_create()
	var size = Vector2(20,20)
	PhysicsServer2D.shape_set_data(shape_rid, size)

	var checks_point = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2: # avoid a bug in first frame
			return
		
		if true: # limit the scope
			p_monitor.add_test("Can collide with Body")
			var body_query := PhysicsShapeQueryParameters2D.new()
			body_query.transform = Transform2D(0, CENTER)
			body_query.collide_with_bodies = true
			body_query.shape_rid = shape_rid
			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
			var collide = true if d_space.intersect_shape(body_query) else false
			p_monitor.add_test_result(collide)
		
		if true: # limit the scope
			p_monitor.add_test("Can  not collide with Body")
			var body_query := PhysicsShapeQueryParameters2D.new()
			body_query.transform = Transform2D(0, CENTER)
			body_query.collide_with_bodies = false
			body_query.shape_rid = shape_rid
			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
			var collide = true if d_space.intersect_shape(body_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Can collide with Area")
			var area_query := PhysicsShapeQueryParameters2D.new()
			area_query.transform = Transform2D(0, CENTER)
			area_query.shape_rid = shape_rid
			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
			area_query.collide_with_areas = true
			var collide = true if d_space.intersect_shape(area_query) else false
			p_monitor.add_test_result(collide)
		
		if true:
			p_monitor.add_test("Can not collide with Area")
			var area_query := PhysicsShapeQueryParameters2D.new()
			area_query.transform = Transform2D(0, CENTER)
			area_query.shape_rid = shape_rid
			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
			area_query.collide_with_areas = false
			var collide = true if d_space.intersect_shape(area_query) else false
			p_monitor.add_test_result(not collide)
			
		if true:
			p_monitor.add_test("Can detects multiple collision")
			var exclude_rid_query := PhysicsShapeQueryParameters2D.new()
			exclude_rid_query.transform = Transform2D(0, TOP_RIGHT)
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.shape_rid = shape_rid
			exclude_rid_query.motion = Vector2(0, BOTTOM_CENTER.y / 0.016)
			var result := d_space.intersect_shape(exclude_rid_query)
			p_monitor.add_test_result(result.size() == 2)
			
		if true:
			p_monitor.add_test("Can limit result multiple collision")
			var exclude_rid_query := PhysicsShapeQueryParameters2D.new()
			exclude_rid_query.transform = Transform2D(0, TOP_RIGHT)
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.shape_rid = shape_rid
			exclude_rid_query.motion = Vector2(0, BOTTOM_CENTER.y / 0.016)
			var result := d_space.intersect_shape(exclude_rid_query, 1)
			p_monitor.add_test_result(result.size() == 1)
			
		# Can exclude a RID
		if true:
			p_monitor.add_test("Can exclude a Body by RID")
			var exclude_rid_query := PhysicsShapeQueryParameters2D.new()
			exclude_rid_query.transform = Transform2D(0, CENTER)
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.shape_rid = shape_rid
			exclude_rid_query.motion = Vector2(CENTER_RIGHT / 0.016)
			exclude_rid_query.exclude = [body.get_rid()]
			var collide = true if d_space.intersect_shape(exclude_rid_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Can exclude an Area by RID")
			var area_query := PhysicsShapeQueryParameters2D.new()
			area_query.transform = Transform2D(0, CENTER)
			area_query.shape_rid = shape_rid
			area_query.collide_with_areas = true
			area_query.exclude = [area.get_rid()]
			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
			var collide = true if d_space.intersect_shape(area_query) else false
			p_monitor.add_test_result(not collide)
			
		if true:
			p_monitor.add_test("Can exclude multiple RID")
			var exclude_rid_query := PhysicsShapeQueryParameters2D.new()
			exclude_rid_query.transform = Transform2D(0, TOP_RIGHT)
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.shape_rid = shape_rid
			exclude_rid_query.motion = Vector2(BOTTOM_CENTER.x / 0.016, 0)
			exclude_rid_query.exclude = [body.get_rid(), body2.get_rid()]
			var collide = true if d_space.intersect_shape(exclude_rid_query) else false
			p_monitor.add_test_result(not collide)
			
		if true:
			p_monitor.add_test("Don't report collision in the wrong collision layer")
			var area_query := PhysicsShapeQueryParameters2D.new()
			area_query.transform = Transform2D(0, CENTER)
			area_query.shape_rid = shape_rid
			area_query.motion = Vector2(-mid_screen_width / 0.016, 0)
			area_query.collide_with_areas = true
			area_query.collision_mask = int(pow(2, 2-1)) # second layer
			var collide = true if d_space.intersect_shape(area_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Report collision in good collision layer")
			var body_query := PhysicsShapeQueryParameters2D.new()
			body_query.transform = Transform2D(0, CENTER)
			body_query.shape_rid = shape_rid
			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
			body_query.collide_with_bodies = true
			body_query.collision_mask = int(pow(2, 2-1)) # second layer
			var collide = true if d_space.intersect_shape(body_query) else false
			p_monitor.add_test_result(collide)
			
		# Rotation
		if true:
			p_monitor.add_test("Can apply rotation to the shape")
			# Without Rotation Don't collide
			var body_query := PhysicsShapeQueryParameters2D.new()
			body_query.shape_rid = shape_rid
			body_query.transform = Transform2D(0, Vector2(CENTER.x, 261-10))
			body_query.motion = Vector2(mid_screen_width / 0.016, 0)
			body_query.collide_with_bodies = true
			var collide1 = true if d_space.intersect_shape(body_query) else false
			# With rotation collide
			var body_query_rot := PhysicsShapeQueryParameters2D.new()
			body_query_rot.shape_rid = shape_rid
			body_query_rot.transform = Transform2D(deg_to_rad(45), Vector2(CENTER.x, 261-10))
			body_query_rot.motion = Vector2(mid_screen_width / 0.016, 0)
			body_query_rot.collide_with_bodies = true
			var collide2 = true if d_space.intersect_shape(body_query_rot) else false
			p_monitor.add_test_result(not collide1 and collide2)
			
		PhysicsServer2D.free_rid(shape_rid)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_point, simulation_duration)

func add_body(p_position: Vector2, p_add_child := true) -> StaticBody2D:
	var body := StaticBody2D.new()
	var body_shape := PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.RECTANGLE, 4)
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
