extends PhysicsUnitTest2D

var simulation_duration := 1.0
var displacement := Vector2(350, 350)
@onready var animatable_tilemap := $TileMap

func test_description() -> String:
	return """When a [Tilemap] has [collision_animatable] turn ON it can be used as a moving platform,
	the body should stay in sync during the movement"""

func test_name() -> String:
	return "Sync to Physics | testing if [collision_animatable] sync body to the Tilemap2D platform"

func test_start() -> void:
	
	animatable_tilemap.collision_animatable = true
	
	# Check x displacement
	var check_pos_x_callback = func(p_body: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		await get_tree().process_frame #sync to physics is applied after the physics frame
		var x_diff = p_body.position.x - 25 - p_monitor.data["platform"].position.x 
		if x_diff > p_body.get_safe_margin() or x_diff < -p_body.get_safe_margin(): # => != 0
			p_monitor.data["failure"] += 1

	# Check y displacement
	var check_pos_y_callback = func(p_body: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		await get_tree().process_frame #sync to physics is applied after the physics frame
		var y_diff = p_body.position.y + 25 - p_monitor.data["platform"].position.y # /!\ here 25
		if y_diff > p_body.get_safe_margin() or y_diff < -p_body.get_safe_margin(): # => != 0
			p_monitor.data["failure"] += 1
		
	var test_lambda = func(_p_target, p_monitor: GenericExpirationMonitor):
		if not p_monitor.data["failure"] == 0:
			p_monitor.error_message = "Out of sync during %d/%d frames" % [p_monitor.data["failure"], p_monitor.frame]
		return p_monitor.data["failure"] == 0
		
	var start_pos: Vector2 = animatable_tilemap.position
	var end_pos := start_pos + displacement
	
	var character := create_character(CENTER + Vector2(25, -25))
	var tween: Tween = get_tree().create_tween()
	tween.set_process_mode(Tween.TWEEN_PROCESS_PHYSICS)
	tween.tween_interval(0.05)
	tween.tween_property(animatable_tilemap, "position", end_pos, simulation_duration/2)
	tween.tween_property(animatable_tilemap, "position", start_pos, simulation_duration/2)
	create_sync_to_physics_monitor("pos x", animatable_tilemap, character, test_lambda, check_pos_x_callback)
	create_sync_to_physics_monitor("pos y", animatable_tilemap, character, test_lambda, check_pos_y_callback)

func create_sync_to_physics_monitor(p_axis: String, p_animator: Node, p_character: CharacterBody2D, p_test:Callable, p_cbk: Callable) -> GenericExpirationMonitor:
	var monitor := create_generic_expiration_monitor(p_character, p_test, p_cbk, simulation_duration)
	monitor.test_name = "Tilemap with [sync_to_physics] %s is sync" % [p_axis]
	monitor.data["platform"] = p_animator
	monitor.data["failure"] = 0
	monitor.success = true
	return monitor

func create_character(p_position: Vector2, p_body_shape := PhysicsTest2D.TestCollisionShape.RECTANGLE) -> CharacterBody2D:
	var character = CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd")
	character.position = p_position
	var body_col: Node2D = PhysicsTest2D.get_default_collision_shape(p_body_shape, 2)
	character.add_child(body_col)
	add_child(character)
	return character
