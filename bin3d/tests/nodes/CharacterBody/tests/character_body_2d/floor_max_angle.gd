extends PhysicsUnitTest2D

var speed := 750
var tolerance = 1.5
var simulation_duration := 1

@onready var spawn_1 := $Spawn1 # max_x < 375 (400-25)
@onready var spawn_2 := $Spawn2 # max_x > 975 (100-25)

func test_description() -> String:
	return """Checks if [floor_max_angle] working properly, the body shoould not detect an angle,
	if the slope angle is greather than [floor_max_angle].
	"""
	
func test_name() -> String:
	return "CharacterBody2D | testing [floor_max_angle]"

func test_start() -> void:
	var test_lambda: Callable = func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		if p_monitor.data["max_angle"] == 44:
			return p_monitor.data["maximum_x"] < 375
		else:
			return p_monitor.data["maximum_x"] > 975
	
	var callback_lambda = func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		if p_target.is_on_floor():
			p_target.velocity = Vector2(speed, 0)
		if p_target.position.x > p_monitor.data["maximum_x"]:
			p_monitor.data["maximum_x"] = p_target.position.x
	
	var character_angle_lower := create_character(spawn_1.position, 44)
	var character_angle_greater := create_character(spawn_2.position, 45)
	
	var monitor_angle_lower := create_generic_expiration_monitor(character_angle_lower, test_lambda, callback_lambda, simulation_duration)
	monitor_angle_lower.test_name = "If [floor_max_angle] is lower than the slope, the body can't climb"
	monitor_angle_lower.data["max_angle"] = 44
	monitor_angle_lower.data["maximum_x"] = 0

	var monitor_angle_greater := create_generic_expiration_monitor(character_angle_greater, test_lambda, callback_lambda, simulation_duration)
	monitor_angle_greater.test_name = "If [floor_max_angle] is greater or equal than the slope, the body can climb"
	monitor_angle_greater.data["max_angle"] = 45
	monitor_angle_greater.data["maximum_x"] = 0
	
func create_character(p_position: Vector2, p_angle_degree: int, p_body_shape := PhysicsTest2D.TestCollisionShape.RECTANGLE) -> CharacterBody2D:
	var character = CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd")
	character.position = p_position
	character.floor_block_on_wall = false
	character.floor_max_angle = deg_to_rad(p_angle_degree)
	character.floor_snap_length = 50
	var body_col: Node2D = get_default_collision_shape(p_body_shape, 2)
	character.add_child(body_col)
	add_child(character)
	return character
