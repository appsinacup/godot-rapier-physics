extends PhysicsUnitTest2D

@export var height := 40
@export var box_size := Vector2(20.0, 20.0)
@export var box_spacing :=  Vector2() #0.5,0.5)
var simulation_duration := 40
var size_boundary := 20
var tolerance := Vector2(0.5, 0.5)

var bodies := []
var top_last_position := Vector2.ZERO

func test_description() -> String:
	return """Checks the stability of the RigidBody Simulation. The pyramid should be stable and the RigidBody
	should sleep after the [simulation_duration]
	"""
	
func test_name() -> String:
	return "RigidBody2D | testing the stability with a pyramid [tolerance %.2v]" % [tolerance]

func test_start() -> void:
	add_collision_bottom(size_boundary)
	create_pyramid()

	var test_sleep: Callable = func(_p_target: PhysicsTest2D, _p_monitor: GenericExpirationMonitor):
		for body_parent in bodies as Array[Node2D]:
			var body: RigidBody2D = body_parent.get_child(0)
			if not body.sleeping:
				return false
		return true
	
	var test_head_position: Callable = func(_p_target: PhysicsTest2D, p_monitor: GenericExpirationMonitor):
		var last_cube_parent: Node2D = bodies[bodies.size() - 1]
		var end_simulation_pos := Vector2(last_cube_parent.get_child(0).position.x, last_cube_parent.position.y)
		if end_simulation_pos.x < top_last_position.x - tolerance.x or end_simulation_pos.x > top_last_position.x + tolerance.x:
			p_monitor.error_message = "Moved too much in x, expected: %v , actual %v" % [top_last_position, end_simulation_pos]
			return false
		if end_simulation_pos.y < top_last_position.y - tolerance.y or end_simulation_pos.y > top_last_position.y + tolerance.y:
			p_monitor.error_message = "Moved too much in y, expected: %v , actual %v" % [top_last_position, end_simulation_pos]
			return false
		
		return true
			
	var pyramid_sleep := create_generic_expiration_monitor(self, test_sleep, null, simulation_duration)
	pyramid_sleep.test_name = "All body are sleep"
	
	var pyramid_top_cube := create_generic_expiration_monitor(self, test_head_position, null, simulation_duration)
	pyramid_top_cube.test_name = "The top cube did not move"
	
func create_pyramid():
	var pos_y = -0.5 * box_size.y - box_spacing.y + Global.WINDOW_SIZE.y - size_boundary
	
	var pos_x
	for level in height:
		var level_index = height - level - 1
		var num_boxes = 2 * level_index + 1

		var row_node = Node2D.new()
		row_node.position = Vector2i(0.0, pos_y)
		row_node.name = "Row%02d" % (level + 1)
		add_child(row_node)
		bodies.append(row_node)

		pos_x = -0.5 * (num_boxes - 1) * (box_size.x + box_spacing.x) + Global.WINDOW_SIZE.x/2
		for box_index in range(num_boxes):
			var box = get_rigid_body()
			box.position = Vector2i(pos_x, 0.0)
			box.name = "Box%02d" % (box_index + 1)
			row_node.add_child(box)

			pos_x += box_size.x + box_spacing.x
		pos_y -= box_size.y + box_spacing.y
	
	top_last_position = Vector2(pos_x - box_size.x + box_spacing.x,  pos_y +  box_size.y - box_spacing.y)
		
func get_rigid_body() -> RigidBody2D:
	var body = RigidBody2D.new()
	var shape = PhysicsTest2D.get_collision_shape(Rect2(Vector2(0, 0), box_size), TestCollisionShape.RECTANGLE, false)
	body.add_child(shape)
	return body
