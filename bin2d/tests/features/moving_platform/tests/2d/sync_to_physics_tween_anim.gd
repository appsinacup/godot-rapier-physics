extends PhysicsUnitTest2D

var simulation_duration := 1.0
var displacement := Vector2(350, 350)

func test_description() -> String:
	return """When [sync_to_physics] is ON, the body should not be one frame behind the plaform.
	This test try to move the body with an [AnimationPlayer] and a [Tween]."""

func test_name() -> String:
	return "Sync to Physics | checks if the body is synced with the platform with Tween/AnimationPlayer"

func test_start() -> void:
	# Check x displacement
	var check_pos_x_callback = func(p_body: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		await get_tree().process_frame #sync to physics is applied after the physics frame
		if p_body.position.x - p_monitor.data["platform"].position.x != 0:
			p_monitor.data["failure"] += 1

	# Check y displacement
	var check_pos_y_callback = func(p_body: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		await get_tree().process_frame #sync to physics is applied after the physics frame
		var y_diff = p_body.position.y + 50 - p_monitor.data["platform"].position.y
		if y_diff > p_body.get_safe_margin() or y_diff < -p_body.get_safe_margin(): # => != 0
			p_monitor.data["failure"] += 1

	var test_lambda = func(_p_target, p_monitor: GenericExpirationMonitor):
		if p_monitor.data["sync_to_physics"]:
			if p_monitor.data["failure"] != 0:
				p_monitor.error_message = "Out of sync during %d frames" % [p_monitor.data["failure"]]
			return p_monitor.data["failure"] == 0
		return p_monitor.data["failure"] != 0

	# Tween Setup
	var platform_tween_from_pos := CENTER - Vector2(Global.WINDOW_SIZE.x /3, 0)

	var character_tween_sync := create_character(platform_tween_from_pos - Vector2(0, 50))
	var animatable_tween_sync := create_platform(true, platform_tween_from_pos, platform_tween_from_pos + displacement, true)

	var animatable_tween_no_sync := create_platform(false, platform_tween_from_pos + Vector2(0, 150),  platform_tween_from_pos + displacement + Vector2(0, 150), true)
	var character_tween_no_sync := create_character(platform_tween_from_pos + Vector2(0, 150) - Vector2(0, 50))

	create_sync_to_physics_monitor("Tween", "pos x", animatable_tween_sync, character_tween_sync, true, test_lambda, check_pos_x_callback)
	create_sync_to_physics_monitor("Tween", "pos x", animatable_tween_no_sync, character_tween_no_sync, false, test_lambda, check_pos_x_callback)
	create_sync_to_physics_monitor("Tween", "pos y", animatable_tween_sync, character_tween_sync, true, test_lambda, check_pos_y_callback)
	create_sync_to_physics_monitor("Tween", "pos y", animatable_tween_no_sync, character_tween_no_sync, false, test_lambda, check_pos_y_callback)

	# Animation Player Setup
	var platform_animp_from_pos := CENTER + Vector2(Global.WINDOW_SIZE.x /3, 0)

	var character_animp_sync := create_character(platform_animp_from_pos - Vector2(0, 50))
	var animatable_animp_sync := create_platform(true, platform_animp_from_pos, platform_animp_from_pos + displacement, false)

	var character_animp_no_sync := create_character(platform_animp_from_pos + Vector2(0, 150)  - Vector2(0, 50))
	var animatable_animp_no_sync := create_platform(false, platform_animp_from_pos + Vector2(0, 150), platform_animp_from_pos + displacement + Vector2(0, 150), false)

	create_sync_to_physics_monitor("Animation Player", "pos x", animatable_animp_sync, character_animp_sync, true, test_lambda, check_pos_x_callback)
	create_sync_to_physics_monitor("Animation Player", "pos x", animatable_animp_no_sync, character_animp_no_sync, false, test_lambda, check_pos_x_callback)
	create_sync_to_physics_monitor("Animation Player", "pos y", animatable_animp_sync, character_animp_sync, true, test_lambda, check_pos_y_callback)
	create_sync_to_physics_monitor("Animation Player", "pos y", animatable_animp_no_sync, character_animp_no_sync, false, test_lambda, check_pos_y_callback)

func create_sync_to_physics_monitor(p_name_animator: String, p_axis: String, p_animator: Node, p_character: CharacterBody2D, p_sync_to_physic: bool, p_test:Callable, p_cbk: Callable):
	var monitor := create_generic_expiration_monitor(p_character, p_test, p_cbk, simulation_duration)
	var txt := "with" if p_sync_to_physic else "without"
	var txt2 := "is sync" if p_sync_to_physic else "is not sync" 
	monitor.test_name = "%s %s [sync_to_physics] %s %s" % [p_name_animator, txt,  p_axis, txt2]
	monitor.data["platform"] = p_animator
	monitor.data["sync_to_physics"] = p_sync_to_physic
	monitor.data["failure"] = 0
	return monitor

func create_platform(p_sync_to_physics:bool, p_platform_start_pos:Vector2, p_platform_end_pos:Vector2, p_tween := true) -> AnimatableBody2D:
	#add platform
	var animatable_body:AnimatableBody2D
	
	animatable_body = AnimatableBody2D.new()
	animatable_body.add_child(PhysicsTest2D.get_collision_shape(Rect2(Vector2(0,0), Vector2(150,50)), PhysicsTest2D.TestCollisionShape.RECTANGLE, false))
	animatable_body.position = p_platform_start_pos
	animatable_body.sync_to_physics = p_sync_to_physics
	add_child(animatable_body)
	
	if p_tween:
		# Tween movement
		var tween: Tween = get_tree().create_tween()
		tween.set_process_mode(Tween.TWEEN_PROCESS_PHYSICS if p_sync_to_physics else Tween.TWEEN_PROCESS_IDLE)
		tween.tween_interval(0.05)
		tween.tween_property(animatable_body, "position", p_platform_end_pos, simulation_duration/2)
		tween.tween_property(animatable_body, "position", p_platform_start_pos, simulation_duration/2)
	else:
		var animation := Animation.new()
		var track_index = animation.add_track(Animation.TYPE_VALUE)
		animation.track_set_path(track_index, "%s:position" % [animatable_body.get_path()])
		animation.track_insert_key(track_index, 0, p_platform_start_pos)
		animation.track_insert_key(track_index, simulation_duration/2, p_platform_end_pos)
		animation.track_insert_key(track_index, simulation_duration, p_platform_start_pos)
		
		var animation_library := AnimationLibrary.new()
		animation_library.add_animation("move_platform", animation)
	
		var animation_player := AnimationPlayer.new()
		animation_player.add_animation_library("test", animation_library)
		animation_player.playback_process_mode = AnimationPlayer.ANIMATION_PROCESS_PHYSICS if p_sync_to_physics else AnimationPlayer.ANIMATION_PROCESS_IDLE
		add_child(animation_player)
		animation_player.play("test/move_platform")
		
	return animatable_body

func create_character(p_position: Vector2, p_body_shape := PhysicsTest2D.TestCollisionShape.RECTANGLE) -> CharacterBody2D:
	var character = CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd")
	character.position = p_position
	var body_col: Node2D = PhysicsTest2D.get_default_collision_shape(p_body_shape, 2)
	character.add_child(body_col)
	add_child(character)
	return character
