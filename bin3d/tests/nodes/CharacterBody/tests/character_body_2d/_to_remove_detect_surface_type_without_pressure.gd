extends PhysicsUnitTest2D

@export var body_mode: CharacterBody2D.MotionMode = CharacterBody2D.MotionMode.MOTION_MODE_GROUNDED
var speed := 1500
var simulation_duration := 2.0

func test_description() -> String:
	return """Checks that the surface is detected (wall, ground, ceiling), without constant preessure,
	only one surface at a time should be detected, and only when the body touches them, not during the movement.
	Param [body_mode]: in Floating mode only wall should be detected.
	"""
	
func test_name() -> String:
	var mode = "Grounded" if body_mode == CharacterBody2D.MOTION_MODE_GROUNDED else "Floating"
	return "CharacterBody2D | testing surface detection when the body don't push against the surface [mode=%s]" % [mode]

func test_start() -> void:
	add_collision_boundaries(150)
	
	# checks all collision type
	var test_lambda
	if body_mode == CharacterBody2D.MOTION_MODE_GROUNDED:
		test_lambda = func(p_step, p_target: CharacterBody2D, p_monitor: GenericStepMonitor):
			if p_step == 0: return p_target.get_slide_collision_count() == 0 # idle
			elif p_step == 1: return p_target.is_on_wall_only() and p_target.get_slide_collision_count() == 1 # touch right wall
			elif p_step == 2: return p_target.get_slide_collision_count() == 0 # go up
			elif p_step == 3: return p_target.is_on_ceiling_only() and p_target.get_slide_collision_count() == 1 # hit the ceiling
			elif p_step == 4: return p_target.get_slide_collision_count() == 0 # move left
			elif p_step == 5: return p_target.is_on_wall_only() and p_target.get_slide_collision_count() == 1 # hit the left wall
			elif p_step == 6: return p_target.get_slide_collision_count() == 0 # move down
			elif p_step == 7: return p_target.is_on_floor_only() and p_target.get_slide_collision_count() == 1 # hit the floor
			elif p_step == 8: return p_target.get_slide_collision_count() == 0 # move right
	elif body_mode == CharacterBody2D.MOTION_MODE_FLOATING:
		test_lambda = func(p_step, p_target: CharacterBody2D, p_monitor: GenericStepMonitor):
			if p_step == 0: return p_target.get_slide_collision_count() == 0 # idle
			elif p_step == 1: return p_target.is_on_wall_only() and p_target.get_slide_collision_count() == 1 # touch right wall
			elif p_step == 2: return p_target.get_slide_collision_count() == 0 # go up
			elif p_step == 3: return p_target.is_on_wall_only() and p_target.get_slide_collision_count() == 1 # hit the ceiling
			elif p_step == 4: return p_target.get_slide_collision_count() == 0 # move left
			elif p_step == 5: return p_target.is_on_wall_only() and p_target.get_slide_collision_count() == 1 # hit the left wall
			elif p_step == 6: return p_target.get_slide_collision_count() == 0 # move down
			elif p_step == 7: return p_target.is_on_wall_only() and p_target.get_slide_collision_count() == 1 # hit the floor
			elif p_step == 8: return p_target.get_slide_collision_count() == 0 # move right
	
	var physics_step_cbk = func(p_step: int, p_target: CharacterBody2D, p_is_transition: bool, p_monitor: Monitor):
		if not p_is_transition: 
			return
		if p_step == 0: p_target.velocity = Vector2(speed, 0) # right
		elif p_step == 1: p_target.velocity = Vector2(0, -speed) # up 
		elif p_step == 3: p_target.velocity = Vector2(-speed, 0) # left
		elif p_step == 5: p_target.velocity = Vector2(0, speed) # down
		elif p_step == 7: p_target.velocity = Vector2(speed, 0) # right

	var cpt_layer := 1
	for shape_type in PhysicsTest2D.TestCollisionShape.values():
		if shape_type == PhysicsTest2D.TestCollisionShape.WORLD_BOUNDARY or shape_type == PhysicsTest2D.TestCollisionShape.CONCAVE_SEGMENT:
			continue
		# Create character
		var character = CharacterBody2D.new()
		character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide.gd")
		character.motion_mode = body_mode
		character.position = CENTER
		character.collision_layer = 0
		character.collision_mask = 0
		character.set_collision_layer_value(cpt_layer, true)
		character.set_collision_mask_value(cpt_layer, true)
		
		var body_col: Node2D = get_default_collision_shape(shape_type, 2)
		character.add_child(body_col)
		
		add_child(character)

		var contact_monitor := create_generic_step_monitor(character, test_lambda, physics_step_cbk, simulation_duration)
		contact_monitor.test_name = "%s detects collisions correctly" % [shape_name(shape_type)]
		
		cpt_layer += 1
