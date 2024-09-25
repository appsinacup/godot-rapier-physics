extends PhysicsUnitTest2D

var simulation_duration := .5

func test_description() -> String:
	return """Checks that [Raycast2D] collisions work correctly with all parameters [exclude_parent],
	[hit_from_inside], [enabled]"""
	
func test_name() -> String:
	return "Raycast2D | testing collision"

func test_start() -> void:

	var ray_lambda = func(p_target: RigidBody2D, p_monitor: Monitor):

		var ray1 = p_target.get_child(1) # from up 
		if ray1.is_colliding():
			p_monitor.error_message = "Raycast not entabled should not collidee"
			return false
		var ray2 = p_target.get_child(2) # from up 
		if not ray2.is_colliding():
			p_monitor.error_message = "Raycast from up to bottom should collide"
			return false
		var ray3 = p_target.get_child(3) # from up excluded
		
		if ray3.is_colliding():
			p_monitor.error_message = "Raycast from up to bottom with [exclude parent] ON should not collide"
			return false
		var ray4 = p_target.get_child(4) # inside
		if not ray4.is_colliding():
			p_monitor.error_message = "Raycast inside with [hit inside] ON should collide"
			return false
		var ray5 = p_target.get_child(5) # inside without hit inside
		if ray5.is_colliding():
			p_monitor.error_message = "Raycast inside with [hit inside] OFF should not collide"
			return false
		
		if p_target.name != "Concave Polygon": # not true because of concave decomposition
			var ray6 = p_target.get_child(6) # from center to bottom
			if ray6.is_colliding():
				p_monitor.error_message = "Raycast from center to bottom with [hit inside] OFF should not collide"
				return false
			
		var ray7 = p_target.get_child(7)
		if not ray7.is_colliding():
			p_monitor.error_message = "Raycast from center to bottom with [hit inside] OFF should collide with the bottom box"
			return false

		return true
	
	var offset := (Global.WINDOW_SIZE.x) / (PhysicsTest2D.TestCollisionShape.values().size() -1)
	var cpt := 0
	var bottom_box := PhysicsTest2D.get_static_body_with_collision_shape(Rect2(Vector2(600, 600), Vector2(850,20)), PhysicsTest2D.TestCollisionShape.RECTANGLE)
	bottom_box.position = Vector2(500, 600)
	add_child(bottom_box)
	for shape_type in PhysicsTest2D.TestCollisionShape.values():
		if shape_type == PhysicsTest2D.TestCollisionShape.WORLD_BOUNDARY or shape_type == PhysicsTest2D.TestCollisionShape.CONCAVE_SEGMENT:
			continue
		var body := create_rigid_body(Vector2(100 + offset * cpt, CENTER.y), shape_type)
		body.name = PhysicsTest2D.shape_name(shape_type)
		var monitor := create_generic_expiration_monitor(body, ray_lambda, null, simulation_duration)
		monitor.test_name = "Testing Raycast collision with %s" % [PhysicsTest2D.shape_name(shape_type)]
		cpt += 1

func create_rigid_body(p_position: Vector2, p_collision_shape: PhysicsTest2D.TestCollisionShape) -> RigidBody2D:
	var body := RigidBody2D.new()
	body.gravity_scale = 0
	body.add_child(PhysicsTest2D.get_default_collision_shape(p_collision_shape, 6))
	body.position = p_position
	add_child(body)
	
	create_raycast(body, Vector2(10, -200), 200, true, false, false) # up from center not enabled
	create_raycast(body, Vector2(-20, -200), 200) # up from center
	create_raycast(body, Vector2(20, -200), 200, false, true) # up from center but exclude parent
	create_raycast(body, Vector2(-20, 10), 10, true) # inside with hit
	create_raycast(body, Vector2(20, 10), 10) # inside without hit
	create_raycast(body, Vector2(-3,0), 200) # center from outside
	create_raycast(body, Vector2(3,0), 300) # center with hit outside
	
	return body

func create_raycast(p_parent: RigidBody2D, p_position: Vector2, p_size: int, p_hit_from_inside := false, p_exclude_parent := false, enabled := true) -> RayCast2D:
	var ray := RayCast2D.new()
	ray.position = p_position
	ray.is_colliding()
	ray.enabled = enabled
	ray.exclude_parent = p_exclude_parent
	ray.target_position = Vector2(0, p_size)
	ray.hit_from_inside = p_hit_from_inside
	p_parent.add_child(ray)
	return ray
