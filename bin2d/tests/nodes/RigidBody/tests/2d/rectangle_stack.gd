extends PhysicsUnitTest2D

var stack_height := 30
var max_horizontal_movement := 5.0
var body_size := Vector2(25, 25)
var body_spacing := 1
var body_shape: PhysicsTest2D.TestCollisionShape = PhysicsTest2D.TestCollisionShape.RECTANGLE
var simulation_duration := 3
var tolerance := 3.3

func test_description() -> String:
	return """Checks the stability of the RigidBody Simulation. The stack should be stable and the RigidBody
	should sleep after the [simulation_duration]
	"""
	
func test_name() -> String:
	return "RigidBody2D | testing the box stack stability"

func test_start() -> void:
	add_collision_boundaries(20, false)

	var stack = Node2D.new()
	stack.position = BOTTOM_CENTER - Vector2(0, 20 + -body_size.y * 0.5)
	
	var bodies_array : Array[RigidBody2D] = []
	for i in range(stack_height):
		var body := RigidBody2D.new()
		var body_col: Node2D = PhysicsTest2D.get_collision_shape(Rect2(Vector2(-body_size.x * 0.5, -body_size.y * 0.5), body_size), body_shape, true)
		body.add_child(body_col)
		
		# Spawn the body
		body.position = Vector2(0, -(body_size.y + body_spacing) * (i+1))
		bodies_array.append(body)
		stack.add_child(body)
	
	add_child(stack)
	
	# 1. Should be sleeping
	var should_be_sleep = func(_p_target: Node2D, _p_monitor: GenericExpirationMonitor):
		for body in bodies_array as Array[RigidBody2D]:
			if not body.sleeping:
				return false
		return true
	
	var sleep_monitor := create_generic_expiration_monitor(stack, should_be_sleep, null, simulation_duration)
	sleep_monitor.test_name = "The bodies are sleeping"
	
	# 2. Should not move horizontally
	var should_not_move_in_x: Callable = func(_p_target: Node2D, p_monitor: GenericExpirationMonitor):
		for body in bodies_array as Array[RigidBody2D]:
			if not (body.position.x > -tolerance and body.position.x < tolerance):
				p_monitor.error_message = "A body moved by %.1f px" % [body.position.x]
				return false
		return true

	var horizontal_monitor := create_generic_expiration_monitor(stack, should_not_move_in_x, null, simulation_duration)
	horizontal_monitor.test_name = "The bodies did not move horizontally more than %.1f px" % [tolerance]
	
	# 3. Should be sorted vertically
	var should_be_sorted_vertically = func(_p_target: Node2D, _p_monitor: GenericExpirationMonitor):
		var child_height = -INF
		for body in bodies_array as Array[RigidBody2D]:
			var height = -body.position.y # easier, because the smaller the y, the higher it is (a bit counter intuitive)
			if height > child_height:
				child_height = height
			else:
				return false
		return true

	var sorted_vertically_monitor := create_generic_expiration_monitor(stack, should_be_sorted_vertically, null, simulation_duration)
	sorted_vertically_monitor.test_name = "The bodies are sorted vertically"

	# 4. Only neighboring children overlap
	var shoud_overlaps_with_neighbours= func(p_target: Node2D, _p_monitor: GenericExpirationMonitor):
		var bodies := bodies_array as Array[RigidBody2D]
		for child_idx in bodies.size():
			var body := bodies[child_idx]
			for shape_owner_id in body.get_shape_owners():
				for shape_id in body.shape_owner_get_shape_count(shape_owner_id):
					var shape = body.shape_owner_get_shape(shape_owner_id, shape_id)
					var parameters := PhysicsShapeQueryParameters2D.new()
					parameters.transform = body.global_transform * body.shape_owner_get_transform(shape_owner_id)
					parameters.set_shape(shape)
					var results = p_target.get_world_2d().direct_space_state.intersect_shape(parameters)
					for result in results:
						if not result.collider is RigidBody2D: # can be the level
							continue
						var sibling_idx = bodies.find(result.collider)
						if sibling_idx == -1 or sibling_idx == child_idx:
							continue
						if abs(child_idx - sibling_idx) > 1: # sibling is not a neighbor
							return false
		return true
		
	var overlaps_with_neighbours_monitor := create_generic_expiration_monitor(stack, shoud_overlaps_with_neighbours, null, simulation_duration)
	overlaps_with_neighbours_monitor.test_name = "Only neighboring children overlap"
