extends PhysicsUnitTest3D

@export var shape: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON
@export var stack_height := 7
@export var body_spacing := 0.1
@export var simulation_duration := 5
@export var tolerance := 0.4

func test_description() -> String:
	return """Checks the stability of the RigidBody Simulation. The stack should be stable and the RigidBody
	should sleep after the [simulation_duration]
	"""
	
func test_name() -> String:
	return "RigidBody3D | testing the %s stack stability" % [PhysicsTest3D.shape_name(shape)]

func test_start() -> void:
	$Camera.current = true
	
	# Ground
	var ground_body := StaticBody3D.new()
	var col_shape = CollisionShape3D.new()
	col_shape.shape = WorldBoundaryShape3D.new()
	ground_body.add_child(col_shape)
	add_child(ground_body)
	
	var stack = Node3D.new()
	stack.position = Vector3(0, -0.5, 0)
	
	var bodies_array : Array[RigidBody3D] = []
	for i in range(stack_height):
		var body := RigidBody3D.new()
		var body_col: Node3D = PhysicsTest3D.get_default_collision_shape(shape)
		body.add_child(body_col)
		
		# Spawn the body
		body.position = Vector3(0, (1 + body_spacing) * (i+1), 0)
		bodies_array.append(body)
		stack.add_child(body)

	add_child(stack)

	# 1. Should be sleeping
	var should_be_sleep = func(_p_target: Node3D, _p_monitor: GenericExpirationMonitor):
		for body in bodies_array as Array[RigidBody3D]:
			if not body.sleeping:
				return false
		return true
	
	var sleep_monitor := create_generic_expiration_monitor(stack, should_be_sleep, null, simulation_duration)
	sleep_monitor.test_name = "The bodies are sleeping"
	
	# 2. Should not move horizontally
	var should_not_move_in_x: Callable = func(_p_target: Node3D, p_monitor: GenericExpirationMonitor):
		for body in bodies_array as Array[RigidBody3D]:
			if not (body.position.x > -tolerance and body.position.x < tolerance):
				p_monitor.error_message = "A body moved by %.2f px" % [body.position.x]
				return false
		return true
	
	var horizontal_monitor := create_generic_expiration_monitor(stack, should_not_move_in_x, null, simulation_duration)
	horizontal_monitor.test_name = "The bodies did not move horizontally more than %.1f px" % [tolerance]
	
	# 3. Should be sorted vertically
	var should_be_sorted_vertically = func(_p_target: Node3D, _p_monitor: GenericExpirationMonitor):
		var child_height = -INF
		for body in bodies_array as Array[RigidBody3D]:
			var height = body.position.y # easier, because the smaller the y, the higher it is (a bit counter intuitive)
			if height > child_height:
				child_height = height
			else:
				return false
		return true
		
	var sorted_vertically_monitor := create_generic_expiration_monitor(stack, should_be_sorted_vertically, null, simulation_duration)
	sorted_vertically_monitor.test_name = "The bodies are sorted vertically"
	
	# 4. Only neighboring children overlap
	var shoud_overlaps_with_neighbours= func(p_target: Node3D, _p_monitor: GenericExpirationMonitor):
		var bodies := bodies_array as Array[RigidBody3D]
		for child_idx in bodies.size():
			var body := bodies[child_idx]
			for shape_owner_id in body.get_shape_owners():
				for shape_id in body.shape_owner_get_shape_count(shape_owner_id):
					var shape_owner = body.shape_owner_get_shape(shape_owner_id, shape_id)
					var parameters := PhysicsShapeQueryParameters3D.new()
					parameters.transform = body.global_transform * body.shape_owner_get_transform(shape_owner_id)
					parameters.set_shape(shape_owner)
					var results = p_target.get_world_3d().direct_space_state.intersect_shape(parameters)
					for result in results:
						if not result.collider is RigidBody3D: # can be the level
							continue
						var sibling_idx = bodies.find(result.collider)
						if sibling_idx == -1 or sibling_idx == child_idx:
							continue
						if abs(child_idx - sibling_idx) > 1: # sibling is not a neighbor
							return false
		return true
		
	var overlaps_with_neighbours_monitor := create_generic_expiration_monitor(stack, shoud_overlaps_with_neighbours, null, simulation_duration)
	overlaps_with_neighbours_monitor.test_name = "Only neighboring children overlap"
	
