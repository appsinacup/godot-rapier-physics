extends PhysicsUnitTest2D

var simulation_duration := 10

const SHAPE_OFFSET := Vector2(40, 0)

func test_description() -> String:
	return """Checks that the space override of an [Area2D] applies once to a [RigidBody2D] with
	several shapes, for as long as any of those shapes overlaps the area
	"""

func test_name() -> String:
	return "Area2D | testing space override on a body with several shapes"

func test_start() -> void:
	var two_shapes: Array[Vector2] = [-SHAPE_OFFSET, SHAPE_OFFSET]
	var one_shape: Array[Vector2] = [Vector2.ZERO]

	# Area without gravity that slides off the body, one shape at a time
	var partial_area_position := Vector2(200, CENTER.y)
	var area_partial := add_area(partial_area_position, Vector2(100, 100))
	area_partial.gravity_space_override = Area2D.SPACE_OVERRIDE_REPLACE
	area_partial.gravity = 0.0
	var body_partial := add_rigid_body(partial_area_position, two_shapes)

	# Area with combined damping, holding a body with one shape and a body with two
	var damp_area_position := Vector2(CENTER.x, CENTER.y)
	var area_damp := add_area(damp_area_position, Vector2(500, 200))
	area_damp.linear_damp_space_override = Area2D.SPACE_OVERRIDE_COMBINE
	area_damp.linear_damp = 2.0
	var body_damp_one_shape := add_rigid_body(damp_area_position + Vector2(-150, -50), one_shape)
	var body_damp_two_shapes := add_rigid_body(damp_area_position + Vector2(-150, 50), two_shapes)

	# Area without gravity that is removed from the tree while the body is inside it
	var removed_area_position := Vector2(Global.WINDOW_SIZE.x - 200, CENTER.y)
	var area_removed := add_area(removed_area_position, Vector2(100, 100))
	area_removed.gravity_space_override = Area2D.SPACE_OVERRIDE_REPLACE
	area_removed.gravity = 0.0
	var body_removed := add_rigid_body(removed_area_position, two_shapes)

	var dt := 1.0/60.0
	var default_gravity : Vector2 = ProjectSettings.get_setting("physics/2d/default_gravity_vector")
	default_gravity *= ProjectSettings.get_setting("physics/2d/default_gravity")
	var initial_velocity := Vector2(150, 0)

	var checks_point = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 2:
			body_partial.gravity_scale = 1.0
			body_removed.gravity_scale = 1.0
			body_damp_one_shape.linear_velocity = initial_velocity
			body_damp_two_shapes.linear_velocity = initial_velocity

			# Only the right shape still overlaps the area
			area_partial.position.x += 70
			remove_child(area_removed)

			p_monitor.data["body_partial_pos"] = body_partial.position
			p_monitor.data["body_removed_pos"] = body_removed.position

		if p_monitor.frame == 22:
			if true: # limit the scope
				p_monitor.add_test("Area keeps its override while one of the body's shapes is inside")
				var pos_start = p_monitor.data["body_partial_pos"]
				var motion = body_partial.position - pos_start
				var expected = Vector2.ZERO
				var success := Utils.vec2_equals(motion, expected, 0.01)
				if not success:
					p_monitor.add_test_error("Override was lifted when the first shape left the area, expected motion %v, got %v" % [expected, motion])
				p_monitor.add_test_result(success)

			if true: # limit the scope
				p_monitor.add_test("Area applies combined damping once to a body with several shapes")
				var velocity_one_shape := body_damp_one_shape.linear_velocity
				var velocity_two_shapes := body_damp_two_shapes.linear_velocity
				var damped := velocity_one_shape.x < initial_velocity.x * 0.9
				var success := damped and Utils.vec2_equals(velocity_two_shapes, velocity_one_shape, 1.0)
				if not damped:
					p_monitor.add_test_error("Area damping was not applied, velocity is still %v" % [velocity_one_shape])
				elif not success:
					p_monitor.add_test_error("Damping was applied once per shape, expected velocity %v, got %v" % [velocity_one_shape, velocity_two_shapes])
				p_monitor.add_test_result(success)

			if true: # limit the scope
				p_monitor.add_test("Area removed from the tree stops overriding the body")
				var pos_start = p_monitor.data["body_removed_pos"]
				var motion = body_removed.position - pos_start
				var time := 20.0 * dt
				var expected = 0.5 * default_gravity * time * time
				var success := Utils.vec2_equals(motion, expected, 4)
				if not success:
					p_monitor.add_test_error("Override outlived the area's removal, expected motion %v, got %v" % [expected, motion])
				p_monitor.add_test_result(success)

			# Neither shape overlaps the area anymore
			area_partial.position.x += 200

		if p_monitor.frame == 25: # the exit takes a few steps to reach the body
			body_partial.linear_velocity = Vector2.ZERO
			p_monitor.data["body_partial_pos"] = body_partial.position

		if p_monitor.frame == 45:
			if true: # limit the scope
				p_monitor.add_test("Area lifts its override once the last of the body's shapes leaves")
				var pos_start = p_monitor.data["body_partial_pos"]
				var motion = body_partial.position - pos_start
				var time := 20.0 * dt
				var expected = 0.5 * default_gravity * time * time
				var success := Utils.vec2_equals(motion, expected, 4)
				if not success:
					p_monitor.add_test_error("Override was kept after the last shape left the area, expected motion %v, got %v" % [expected, motion])
				p_monitor.add_test_result(success)

			area_removed.queue_free()
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_point, simulation_duration)

func add_rigid_body(p_position: Vector2, p_shape_offsets: Array[Vector2]) -> RigidBody2D:
	var body := RigidBody2D.new()
	for offset in p_shape_offsets:
		var body_shape := PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.CIRCLE)
		body_shape.position = offset
		body.add_child(body_shape)
	body.position = p_position
	body.gravity_scale = 0.0
	body.can_sleep = false
	add_child(body)
	return body

func add_area(p_position: Vector2, p_size: Vector2) -> Area2D:
	var area := Area2D.new()
	area.add_child(PhysicsTest2D.get_collision_shape(Rect2(Vector2.ZERO, p_size), PhysicsTest2D.TestCollisionShape.RECTANGLE))
	area.position = p_position
	add_child(area)
	return area
