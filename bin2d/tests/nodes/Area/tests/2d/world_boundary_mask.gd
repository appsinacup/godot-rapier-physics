extends PhysicsUnitTest2D

var simulation_duration := 10

func test_description() -> String:
	return """Checks that an [Area2D] with a [WorldBoundaryShape2D] only detects bodies on a layer
	its mask covers, while it overlaps a [StaticBody2D] that also uses a [WorldBoundaryShape2D]
	"""

func test_name() -> String:
	return "Area2D | testing the collision mask with world boundaries"

func test_start() -> void:
	var area := Area2D.new()
	add_world_boundary(area, CENTER)

	add_world_boundary(StaticBody2D.new(), CENTER + Vector2(0, 200))

	# The body's mask covers the area's layer, but the area's mask does not cover the body's layer.
	var masked_body_inside := add_rigid_body(CENTER + Vector2(-100, 100), 2, 1)
	masked_body_inside.gravity_scale = 0.0
	var masked_body_dropped := add_rigid_body(CENTER + Vector2(0, -100), 2, 1)
	var detected_body := add_rigid_body(CENTER + Vector2(100, 100), 1, 1)
	detected_body.gravity_scale = 0.0

	var masked_entered := [false]
	area.body_entered.connect(func(body: Node2D) -> void:
		if body == masked_body_inside or body == masked_body_dropped:
			masked_entered[0] = true
	)

	var checks_point = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 60: # let the dropped body fall into the area
			return

		if true:
			p_monitor.add_test("Detect a body on a layer the mask covers")
			p_monitor.add_test_result(area.overlaps_body(detected_body))

		if true:
			p_monitor.add_test("Don't detect a body placed inside on a layer the mask doesn't cover")
			p_monitor.add_test_result(!area.overlaps_body(masked_body_inside))

		if true:
			p_monitor.add_test("Don't detect a body dropped inside on a layer the mask doesn't cover")
			var inside := masked_body_dropped.global_position.y > CENTER.y
			if !inside:
				p_monitor.add_test_error("The dropped body did not reach the area.")
			p_monitor.add_test_result(inside and !area.overlaps_body(masked_body_dropped))

		if true:
			p_monitor.add_test("Don't emit body_entered for bodies the mask doesn't cover")
			p_monitor.add_test_result(!masked_entered[0])

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_point, simulation_duration)

func add_world_boundary(p_object: CollisionObject2D, p_position: Vector2) -> void:
	var shape := CollisionShape2D.new()
	shape.shape = WorldBoundaryShape2D.new()
	p_object.add_child(shape)
	p_object.position = p_position
	add_child(p_object)

func add_rigid_body(p_position: Vector2, p_collision_layer: int, p_collision_mask: int) -> RigidBody2D:
	var body := RigidBody2D.new()
	body.add_child(PhysicsTest2D.get_default_collision_shape(PhysicsTest2D.TestCollisionShape.CIRCLE))
	body.position = p_position
	body.collision_layer = p_collision_layer
	body.collision_mask = p_collision_mask
	add_child(body)
	return body
