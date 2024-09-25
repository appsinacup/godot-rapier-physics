extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks all the [PhysicsPointQueryParameters2D] of [intersect_point]
	"""
	
func test_name() -> String:
	return "DirectSpaceState2D | testing [intersect_point] from [get_world_2d().direct_space_state]"
	
func test_start() -> void:

	# Add Area on the LEFT
	var area := add_area(CENTER_LEFT)
	
	# Add Body on the RIGHT
	var body := add_body(CENTER_RIGHT)
	body.set_collision_layer_value(2, true)
	body.set_collision_mask_value(2, true)
	
	# Add two body in the center
	add_area(CENTER)
	add_body(CENTER)
	add_area(CENTER)
	add_body(CENTER)
	
	# Add Canvas Layers
	var canvas := CanvasLayer.new()
	canvas.layer = 1
	var canvas_area := add_area(TOP_CENTER, false)
	canvas.add_child(canvas_area)
	add_child(canvas)
	
	var canvas2 := CanvasLayer.new()
	canvas2.layer = 2
	var canvas_area2 := add_area(TOP_CENTER, false)
	canvas2.add_child(canvas_area2)
	add_child(canvas2)
	
	var canvas3 := CanvasLayer.new()
	canvas3.layer = 3
	var canvas_area3 := add_area(TOP_CENTER, false)
	canvas3.add_child(canvas_area3)
	add_child(canvas3)
	
	var canvas_empty := CanvasLayer.new()
	canvas_empty.layer = 4
	add_child(canvas_empty)
	
	var d_space := get_world_2d().direct_space_state
	var checks_point = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2: # avoid a bug in first frame
			return
		
		if true: # limit the scope
			p_monitor.add_test("Don't collide at 1px left from the body")
			var body_query := PhysicsPointQueryParameters2D.new()
			body_query.position = CENTER_RIGHT - (Vector2(51,0)) # Rectangle is 100px wide
			body_query.collide_with_bodies = true
			var result := d_space.intersect_point(body_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)

		if true:
			p_monitor.add_test("Can collide with Body at the left border")
			var body_query := PhysicsPointQueryParameters2D.new()
			body_query.position = CENTER_RIGHT - (Vector2(50,0))
			body_query.collide_with_bodies = true
			var result := d_space.intersect_point(body_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 1)

		if true:
			p_monitor.add_test("Can not collide with Body")
			var body_query := PhysicsPointQueryParameters2D.new()
			body_query.position = CENTER_RIGHT
			body_query.collide_with_bodies = false
			var result := d_space.intersect_point(body_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)

		if true:
			p_monitor.add_test("Can collide with Area")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = CENTER_LEFT
			area_query.collide_with_areas = true
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 1)

		if true:
			p_monitor.add_test("Can not collide with Area")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = CENTER_LEFT
			area_query.collide_with_areas = false
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)

		# Can exclude a RID
		if true:
			p_monitor.add_test("Can exclude a Body by RID")
			var exclude_rid_query := PhysicsPointQueryParameters2D.new()
			exclude_rid_query.position = CENTER_RIGHT
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.exclude = [body.get_rid()]
			var result := d_space.intersect_point(exclude_rid_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)

		if true:
			p_monitor.add_test("Can exclude an Area by RID")
			var exclude_rid_query := PhysicsPointQueryParameters2D.new()
			exclude_rid_query.position = CENTER_LEFT
			exclude_rid_query.collide_with_areas = true
			exclude_rid_query.exclude = [area.get_rid()]
			var result := d_space.intersect_point(exclude_rid_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)
			
		# Canvas Layer
		if true:
			p_monitor.add_test("Should not detect a collision outside the canvas")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = CENTER_LEFT
			area_query.collide_with_areas = true
			area_query.canvas_instance_id = canvas.get_instance_id()
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)
			
		if true:
			p_monitor.add_test("Should not detect a collision inside the empty canvas")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = TOP_CENTER
			area_query.collide_with_areas = true
			area_query.canvas_instance_id = canvas_empty.get_instance_id()
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)
			
		if true:
			p_monitor.add_test("Should detect one collision inside the canvas")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = TOP_CENTER
			area_query.collide_with_areas = true
			area_query.canvas_instance_id = canvas2.get_instance_id()
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			var correct_result = false
			if result_count > 1:
				p_monitor.add_test_error("Found too many results.")
			for index in range(result_count):
				var collider: CollisionObject2D = result[index].collider
				if collider.get_canvas() == canvas2.get_canvas():
					correct_result = true
				else:
					p_monitor.add_test_error("Found a result in the wrong canvas layer.")
			p_monitor.add_test_result(result_count == 1 && correct_result)
			
		if true:
			p_monitor.add_test("Can detect multiple collision")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = CENTER
			area_query.collide_with_bodies = true
			area_query.collide_with_areas = true
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 4)
			
		if true:
			p_monitor.add_test("Can limit multiple collision")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = CENTER
			area_query.collide_with_bodies = true
			area_query.collide_with_areas = true
			var result := d_space.intersect_point(area_query, 2)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 2)
			
		if true:
			p_monitor.add_test("Don't report collision in the wrong collision layer")
			var area_query := PhysicsPointQueryParameters2D.new()
			area_query.position = CENTER_LEFT
			area_query.collide_with_areas = true
			area_query.collision_mask = int(pow(2, 2-1)) # second layer
			var result := d_space.intersect_point(area_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 0)
			
		if true:
			p_monitor.add_test("Report collision in good collision layer")
			var body_query := PhysicsPointQueryParameters2D.new()
			body_query.position = CENTER_RIGHT # Rectangle is 100px wide
			body_query.collide_with_bodies = true
			body_query.collision_mask = int(pow(2, 2-1)) # second layer
			var result := d_space.intersect_point(body_query)
			var result_count = result.size() if result else 0
			p_monitor.add_test_result(result_count == 1)
			
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

