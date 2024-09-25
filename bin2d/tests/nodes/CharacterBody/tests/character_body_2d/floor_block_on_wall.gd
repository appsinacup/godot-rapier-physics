extends PhysicsUnitTest2D

var speed := 750
var tolerance = 1.5
var simulation_duration := 1


@onready var spawn_1 := $Spawn1 # max_x == 275 (300-25)
@onready var spawn_2 := $Spawn2 # max_x > 875 (900-25)

func test_description() -> String:
	return """Checks that the body is stop when is [floor_block_on_wall] is ON,
	without this option the body can climb a bit on a wall, causing jitter.
	"""
	
func test_name() -> String:
	return "CharacterBody2D | testing [floor_block_on_wall]"

func test_start() -> void:
	var test_lambda: Callable = func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		if p_target.floor_block_on_wall:
			return p_monitor.data["maximum_x"] > (275 - p_target.safe_margin) and p_monitor.data["maximum_x"] < (275 + p_target.safe_margin)
		return p_monitor.data["maximum_x"] > (875 + p_target.safe_margin)
	
	var callback_lambda = func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		if p_target.is_on_floor():
			p_target.velocity = Vector2(speed, 0)
		if p_target.position.x > p_monitor.data["maximum_x"]:
			p_monitor.data["maximum_x"] = p_target.position.x
	
	var character_blocked := create_character(spawn_1.position, true)
	var character_not_blocked := create_character(spawn_2.position, false)
	
	var monitor_blocked := create_generic_expiration_monitor(character_blocked, test_lambda, callback_lambda, simulation_duration)
	monitor_blocked.test_name = "The body is properly blocked when [floor_block_on_wall] is ON"
	monitor_blocked.data["maximum_x"] = 0
	
	var monitor_not_blocked := create_generic_expiration_monitor(character_not_blocked, test_lambda, callback_lambda, simulation_duration)
	monitor_not_blocked.test_name = "The body is not blocked when [floor_block_on_wall] is OFF"
	monitor_not_blocked.data["maximum_x"] = 0
	
func create_character(p_position: Vector2, p_block_on_wall, p_body_shape := PhysicsTest2D.TestCollisionShape.RECTANGLE) -> CharacterBody2D:
	var character = CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd")
	character.position = p_position
	character.floor_block_on_wall = p_block_on_wall
	character.floor_max_angle = deg_to_rad(44)
	var body_col: Node2D = get_default_collision_shape(p_body_shape, 2)
	character.add_child(body_col)
	add_child(character)
	return character
