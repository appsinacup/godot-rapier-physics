extends PhysicsUnitTest2D

var simulation_duration := 1

func test_description() -> String:
	return """Checks that [one_way_collision_margin] controls how far a body can enter a one-way shape before passing through.
	"""

func test_name() -> String:
	return "CollisionShape2D | testing [One Way Collision Margin]"

func test_start() -> void:
	var platform := StaticBody2D.new()
	platform.position = CENTER
	var platform_shape := CollisionShape2D.new()
	var platform_rectangle := RectangleShape2D.new()
	platform_rectangle.size = Vector2(200, 20)
	platform_shape.shape = platform_rectangle
	platform_shape.one_way_collision = true
	platform_shape.one_way_collision_margin = 1.0
	platform.add_child(platform_shape)
	add_child(platform)

	var character := CharacterBody2D.new()
	character.position = CENTER + Vector2(0, -18.9)
	var character_shape := CollisionShape2D.new()
	var character_rectangle := RectangleShape2D.new()
	character_rectangle.size = Vector2(20, 20)
	character_shape.shape = character_rectangle
	character.add_child(character_shape)
	add_child(character)

	var checks_margin = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 2:
			p_monitor.add_test("Body past one-way margin can move through shape")
			var collision := character.move_and_collide(Vector2.DOWN * 8.0)
			var success := collision == null
			if not success:
				p_monitor.add_test_error("Expected no collision after entering past one-way margin.")
			p_monitor.add_test_result(success)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_margin, simulation_duration)
