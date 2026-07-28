extends PhysicsUnitTest2D

var simulation_duration := 5
var speed := 300.0
var frames_to_cross := 100
# Vertical offsets of the body centre relative to the platform centre. The body straddles the
# platform's centre line in every case, so a test based on the two shape origins flips its verdict
# between them, while Godot lets the body through for all of them.
var offsets := [-5.0, 0.0, 5.0]

var characters: Array[CharacterBody2D] = []
var target_x := 0.0

func test_description() -> String:
	return """Checks that a one-way shape is only solid along its one-way direction.
	A body walking straight into the platform's side must pass through whether its centre sits
	above or below the platform's centre, which is what Godot's physics does.
	"""

func test_name() -> String:
	return "CollisionShape2D | testing [One Way Collision] is ignored from the sides"

func test_start() -> void:
	var start_x := CENTER.x - 220.0
	target_x = CENTER.x + 180.0
	for i in offsets.size():
		var platform := StaticBody2D.new()
		platform.position = Vector2(CENTER.x, 120.0 + i * 180.0)
		var platform_shape := CollisionShape2D.new()
		var platform_rectangle := RectangleShape2D.new()
		platform_rectangle.size = Vector2(200, 20)
		platform_shape.shape = platform_rectangle
		platform_shape.one_way_collision = true
		platform.add_child(platform_shape)
		platform.collision_layer = 0
		platform.collision_mask = 0
		platform.set_collision_layer_value(i + 1, true)
		add_child(platform)

		var character := CharacterBody2D.new()
		character.position = Vector2(start_x, platform.position.y + offsets[i])
		var character_shape := CollisionShape2D.new()
		var character_rectangle := RectangleShape2D.new()
		character_rectangle.size = Vector2(20, 40)
		character_shape.shape = character_rectangle
		character.add_child(character_shape)
		character.collision_layer = 0
		character.collision_mask = 0
		character.set_collision_mask_value(i + 1, true)
		add_child(character)
		characters.append(character)

	var checks_sides = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		for character in characters:
			character.velocity = Vector2(speed, 0)
			character.move_and_slide()
		if p_monitor.frame < frames_to_cross:
			return
		for i in offsets.size():
			p_monitor.add_test("Walks through the side with a %+.0f px vertical offset" % [offsets[i]])
			var reached: float = characters[i].position.x
			var success := reached >= target_x
			if not success:
				p_monitor.add_test_error("Blocked at x=%.1f, expected to reach x=%.1f." % [reached, target_x])
			p_monitor.add_test_result(success)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_sides, simulation_duration)
