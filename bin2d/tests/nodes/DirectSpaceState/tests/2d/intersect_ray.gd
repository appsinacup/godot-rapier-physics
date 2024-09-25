extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks all the [PhysicsRayQueryParameters2D] of [intersect_ray]
	"""
	
func test_name() -> String:
	return "DirectSpaceState2D | testing [intersect_ray] from [get_world_2d().direct_space_state]"
	
func test_start() -> void:

	# Add Area
	var area := Area2D.new()
	area.add_child(PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.RECTANGLE, 4))
	area.position = CENTER_LEFT
	add_child(area)
	
	# Add Body
	var body := StaticBody2D.new()
	var body_shape := PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.RECTANGLE, 4)
	body.set_collision_layer_value(2, true)
	body.set_collision_mask_value(2, true)
	body.add_child(body_shape)
	body.position = CENTER_RIGHT
	add_child(body)
	
	var d_space := get_world_2d().direct_space_state
	var checks_ray = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2: # avoid a bug in first frame
			return
			
		if true: # limit the scope
			p_monitor.add_test("Can collide with Body")
			var body_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_RIGHT)
			body_query.collide_with_bodies = true
			var collide = true if d_space.intersect_ray(body_query) else false
			p_monitor.add_test_result(collide)
			
		if true:
			p_monitor.add_test("Can not collide with Body")
			var body_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_RIGHT)
			body_query.collide_with_bodies = false
			var collide = true if d_space.intersect_ray(body_query) else false
			p_monitor.add_test_result(not collide)
		
		if true:
			p_monitor.add_test("Can collide with Area")
			var area_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_LEFT)
			area_query.collide_with_areas = true
			var collide = true if d_space.intersect_ray(area_query) else false
			p_monitor.add_test_result(collide)
		
		if true:
			p_monitor.add_test("Can not collide with Area")
			var area_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_LEFT)
			area_query.collide_with_areas = false
			var collide = true if d_space.intersect_ray(area_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Detects Hit from inside")
			var hit_inside_query := PhysicsRayQueryParameters2D.create(CENTER_RIGHT, CENTER)
			hit_inside_query.collide_with_bodies = true
			hit_inside_query.hit_from_inside = true
			var collide = true if d_space.intersect_ray(hit_inside_query) else false
			p_monitor.add_test_result(collide)

		if true:
			p_monitor.add_test("Can NOT detects Hit from inside")
			var hit_inside_query := PhysicsRayQueryParameters2D.create(CENTER_RIGHT, CENTER)
			hit_inside_query.collide_with_bodies = true
			hit_inside_query.hit_from_inside = false
			var collide = true if d_space.intersect_ray(hit_inside_query) else false
			p_monitor.add_test_result(not collide)

		# Can exclude a RID
		if true:
			p_monitor.add_test("Can exclude a Body by RID")
			var exclude_rid_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_RIGHT)
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.exclude = [body.get_rid()]
			var collide = true if d_space.intersect_ray(exclude_rid_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Can exclude an Area by RID")
			var exclude_rid_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_LEFT)
			exclude_rid_query.collide_with_areas = true
			exclude_rid_query.exclude = [area.get_rid()]
			var collide = true if d_space.intersect_ray(exclude_rid_query) else false
			p_monitor.add_test_result(not collide)
			
		if true:
			p_monitor.add_test("Can exclude multiple RID")
			var exclude_rid_query := PhysicsRayQueryParameters2D.create(CENTER_LEFT, CENTER_RIGHT)
			exclude_rid_query.collide_with_areas = true
			exclude_rid_query.collide_with_bodies = true
			exclude_rid_query.hit_from_inside = true
			exclude_rid_query.exclude = [body.get_rid(), area.get_rid()]
			var collide = true if d_space.intersect_ray(exclude_rid_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Don't report collision in the wrong collision layer")
			var area_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_LEFT)
			area_query.collide_with_areas = true
			area_query.collision_mask = int(pow(2, 2-1)) # second layer
			var collide = true if d_space.intersect_ray(area_query) else false
			p_monitor.add_test_result(not collide)

		if true:
			p_monitor.add_test("Report collision in good collision layer")
			var body_query := PhysicsRayQueryParameters2D.create(CENTER, CENTER_RIGHT)
			body_query.collide_with_bodies = true
			body_query.collision_mask = int(pow(2, 2-1)) # second layer
			var collide = true if d_space.intersect_ray(body_query) else false
			p_monitor.add_test_result(collide)
			
		p_monitor.monitor_completed()

	var _check_max_stability_monitor := create_generic_manual_monitor(self, checks_ray, simulation_duration)
