extends PhysicsUnitTest2D

const CHARACTER_SIZE := Vector2(40.0, 40.0)
const PLATFORM_SIZE := Vector2(40.0, 90.0)
const PLATFORM_SPEED := 300.0
const SIMULATION_DURATION := 0.55

var platform: AnimatableBody2D
var character_start_x := 0.0
var platform_start_x := 0.0
var moving := false


func test_description() -> String:
	return """Checks that a moving AnimatableBody2D pushes a stationary CharacterBody2D.
	The character has zero velocity and only calls move_and_slide().
	"""


func test_name() -> String:
	return "CharacterBody2D | animatable body pushes stationary character"


func test_start() -> void:
	character_start_x = CENTER.x
	platform_start_x = CENTER.x - 90.0

	platform = AnimatableBody2D.new()
	platform.sync_to_physics = true
	platform.position = Vector2(platform_start_x, CENTER.y)
	platform.add_child(PhysicsTest2D.get_collision_shape(Rect2(Vector2.ZERO, PLATFORM_SIZE), PhysicsTest2D.TestCollisionShape.RECTANGLE))
	add_child(platform)

	var character := CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide.gd")
	character.motion_mode = CharacterBody2D.MOTION_MODE_FLOATING
	character.position = Vector2(character_start_x, CENTER.y)
	character.velocity = Vector2.ZERO
	character.add_child(PhysicsTest2D.get_collision_shape(Rect2(Vector2.ZERO, CHARACTER_SIZE), PhysicsTest2D.TestCollisionShape.RECTANGLE))
	add_child(character)

	moving = true
	var monitor := create_generic_expiration_monitor(
		character,
		Callable(self, "_is_animatable_push_test_passed"),
		null,
		SIMULATION_DURATION
	)
	monitor.test_name = "Moving animatable body pushes zero-velocity character"


func _physics_process(delta: float) -> void:
	if moving and platform:
		platform.position.x += PLATFORM_SPEED * delta


func _is_animatable_push_test_passed(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor) -> bool:
	moving = false
	var platform_crossed := platform.position.x > character_start_x + 40.0
	var character_pushed := p_target.position.x > character_start_x + 25.0
	if not platform_crossed:
		p_monitor.error_message = "Animatable body did not cross the test body far enough: platform x %f" % platform.position.x
	elif not character_pushed:
		p_monitor.error_message = "Character was not pushed by animatable body: start x %f, final x %f" % [character_start_x, p_target.position.x]
	return platform_crossed and character_pushed
