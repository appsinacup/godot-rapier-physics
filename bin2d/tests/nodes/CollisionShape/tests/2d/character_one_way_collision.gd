extends PhysicsUnitTest2D

@export var body_shape: PhysicsTest2D.TestCollisionShape = TestCollisionShape.CAPSULE

# Distance from the platform centre the body starts at, and how far it travels. The body has to
# cross the centre to count as having passed through, and it must stay well inside its own grid
# cell so it can never reach the platform one row away that shares its collision layer.
const START_OFFSET := 22.0
const SPEED := 60.0
const TRAVEL_FRAMES := 40

var simulation_duration := 5

var bodies: Array[CharacterBody2D] = []
var centers: Array[Vector2] = []
var approach: Array[Vector2] = []
var from_blocked_side: Array[bool] = []
var labels: Array[Label] = []

func test_description() -> String:
	return """Checks if [One Way Collision] works properly with CharacterBody2D.
	Each platform is approached along its own one-way axis, so the contact is face-on for every
	rotation. Coming from the side the one-way direction points away from must be blocked, and
	coming from the opposite side must pass straight through.
	"""

func test_name() -> String:
	return "CollisionShape2D | testing [One Way Collision] with CharacterBody2D"

func test_start() -> void:
	var offset_x := (Global.WINDOW_SIZE.x - 57) / 29.0 # 30 columns
	var offset_y := (Global.WINDOW_SIZE.y - 72) / 11.0 # 12 rows
	var deg := 0
	for y in range(12):
		for x in range(30):
			var center := Vector2(28 + offset_x * x, 36 + offset_y * y)
			# The one-way direction is the shape's local down axis, rotated with the platform.
			var axis := Vector2(0, 1).rotated(deg_to_rad(deg))
			# Alternate sides so the sweep covers both the blocked and the free approach.
			var blocked: bool = deg % 2 == 0

			var platform := PhysicsTest2D.get_static_body_with_collision_shape(
				Rect2(Vector2(0, 0), Vector2(20, 20)), TestCollisionShape.RECTANGLE)
			platform.position = center
			var collision_shape: CollisionShape2D = platform.get_child(0)
			collision_shape.one_way_collision = true
			platform.rotate(deg_to_rad(deg))
			platform.collision_layer = 0
			platform.collision_mask = 0
			platform.set_collision_layer_value(x + 1, true)
			add_child(platform)

			var label := Label.new()
			label.text = "%d°" % [deg]
			label.position = center + Vector2(0, -28)
			label.set("theme_override_font_sizes/font_size", 8)
			add_child(label)
			labels.append(label)

			var character := CharacterBody2D.new()
			character.add_child(PhysicsTest2D.get_default_collision_shape(body_shape, 0.5))
			character.position = center + (-axis if blocked else axis) * START_OFFSET
			character.collision_layer = 0
			character.collision_mask = 0
			character.set_collision_mask_value(x + 1, true)
			add_child(character)

			bodies.append(character)
			centers.append(center)
			approach.append(axis if blocked else -axis)
			from_blocked_side.append(blocked)
			deg += 1

	var checks_one_way = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		var delta := get_physics_process_delta_time()
		for i in bodies.size():
			bodies[i].move_and_collide(approach[i] * SPEED * delta)
		if p_monitor.frame < TRAVEL_FRAMES:
			return

		var blocked_errors := 0
		var passed_errors := 0
		for i in bodies.size():
			var crossed: bool = (bodies[i].position - centers[i]).dot(approach[i]) > 0.0
			if crossed == from_blocked_side[i]:
				labels[i].set("theme_override_colors/font_color", Color.RED)
				if from_blocked_side[i]:
					blocked_errors += 1
				else:
					passed_errors += 1

		p_monitor.add_test("Blocked from the side the one-way direction faces away from")
		if blocked_errors > 0:
			p_monitor.add_test_error("%d rotations let the body through the solid side." % [blocked_errors])
		p_monitor.add_test_result(blocked_errors == 0)

		p_monitor.add_test("Passes through from the other side")
		if passed_errors > 0:
			p_monitor.add_test_error("%d rotations blocked the body on the open side." % [passed_errors])
		p_monitor.add_test_result(passed_errors == 0)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_one_way, simulation_duration)
