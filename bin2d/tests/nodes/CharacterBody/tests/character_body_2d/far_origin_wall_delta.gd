extends PhysicsUnitTest2D

const START_X := 2085.0
const BODY_SIZE := 50.0
const WALL_SIZE := Vector2(40.0, 260.0)
const WALL_LEFT := START_X + 130.0
const SPEED := 250.0
const SIMULATION_DURATION := 0.6


func test_description() -> String:
	return """Checks perpendicular CharacterBody2D motion into a StaticBody2D far from origin.
	The blocked body should report zero position delta, not sub-pixel penetration.
	"""


func test_name() -> String:
	return "CharacterBody2D | far-origin wall push keeps zero delta"


func test_start() -> void:
	var wall := PhysicsTest2D.get_static_body_with_collision_shape(
		Rect2(Vector2(WALL_LEFT, CENTER.y - WALL_SIZE.y * 0.5), WALL_SIZE),
		PhysicsTest2D.TestCollisionShape.RECTANGLE,
		true
	)
	add_child(wall)

	var character := CharacterBody2D.new()
	character.script = load(
		"res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide.gd"
	)
	character.motion_mode = CharacterBody2D.MOTION_MODE_FLOATING
	character.position = Vector2(WALL_LEFT - BODY_SIZE * 0.5 - 12.0, CENTER.y)
	character.velocity = Vector2(SPEED, 0.0)
	var body_shape := PhysicsTest2D.get_default_collision_shape(
		PhysicsTest2D.TestCollisionShape.RECTANGLE,
		2.0
	)
	character.add_child(body_shape)
	add_child(character)

	var monitor := create_generic_expiration_monitor(
		character,
		Callable(self, "_is_far_origin_delta_test_passed"),
		Callable(self, "_on_far_origin_delta_physics_step"),
		SIMULATION_DURATION
	)
	monitor.test_name = "Perpendicular wall push far from origin reports zero blocked delta"
	monitor.data["blocked_steps"] = 0
	monitor.data["bad_delta"] = ""
	monitor.data["max_x"] = -1000000000.0
	monitor.data["expected_stop_x"] = WALL_LEFT - BODY_SIZE * 0.5


func _is_far_origin_delta_test_passed(p_target, p_monitor) -> bool:
	var stopped_before_wall = (
		p_monitor.data["max_x"] <= p_monitor.data["expected_stop_x"] + p_target.safe_margin + 0.01
	)
	return (
		p_monitor.data["blocked_steps"] > 3
		and p_monitor.data["bad_delta"] == ""
		and stopped_before_wall
	)


func _on_far_origin_delta_physics_step(p_target, p_monitor) -> void:
	p_target.velocity = Vector2(SPEED, 0.0)
	p_monitor.data["max_x"] = max(p_monitor.data["max_x"], p_target.position.x)
	if p_target.get_slide_collision_count() == 0 and not p_target.is_on_wall():
		return
	p_monitor.data["blocked_steps"] += 1
	if p_monitor.data["blocked_steps"] == 1:
		return
	var delta = p_target.get_position_delta()
	if not delta.is_zero_approx() and p_monitor.data["bad_delta"] == "":
		p_monitor.data["bad_delta"] = "%v" % delta
		p_monitor.error_message = "Expected zero position delta while blocked, got %v" % delta
