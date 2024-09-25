extends PhysicsUnitTest2D

var speed := 1500
var simulation_duration := 2.0

func test_description() -> String:
	return """Checks that the surface is detected (wall, ground, ceiling), if the body maintains pressure,
	we should have 2 surfaces detected in the corners and one otherwise.
	"""

func test_name() -> String:
	return "CharacterBody2D | testing surface detection when the body constantly push against the surface"

func test_start() -> void:
	add_collision_boundaries(150)
	
	# checks all collision type
	var test_lambda = func(p_step: int, p_body: CharacterBody2D, _p_monitor: GenericStepMonitor):
			if p_step == 0: return p_body.get_slide_collision_count() == 0
			elif p_step == 1: return p_body.is_on_wall_only()
			elif p_step == 2: return p_body.is_on_wall() and p_body.is_on_ceiling()
			elif p_step == 3: return p_body.is_on_ceiling_only()
			elif p_step == 4: return p_body.is_on_wall() and p_body.is_on_ceiling()
			elif p_step == 5: return p_body.is_on_wall_only()
			elif p_step == 6: return p_body.is_on_wall() and p_body.is_on_floor()
			elif p_step == 7: return p_body.is_on_floor_only()
	
	var physics_step_cbk = func(p_step: int, p_body: CharacterBody2D, _p_is_transition: bool, _p_monitor: GenericStepMonitor):
		if p_step == 0: p_body.velocity = Vector2(speed, 0) # right
		elif p_step < 2: p_body.velocity = Vector2(speed, -speed) # up right
		elif p_step < 4: p_body.velocity = Vector2(-speed, -speed) # up left
		elif p_step < 6: p_body.velocity = Vector2(-speed, speed) # down left
		elif p_step == 6: p_body.velocity = Vector2(speed, speed) # down right
	
	var cpt_layer := 1
	for shape_type in PhysicsTest2D.TestCollisionShape.values():
		if shape_type == PhysicsTest2D.TestCollisionShape.WORLD_BOUNDARY or shape_type == PhysicsTest2D.TestCollisionShape.CONCAVE_SEGMENT:
			continue
		# Create character
		var character = CharacterBody2D.new()
		character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide.gd")
		character.motion_mode = CharacterBody2D.MOTION_MODE_GROUNDED
		character.position = CENTER
		character.collision_layer = 0
		character.collision_mask = 0
		character.set_collision_layer_value(cpt_layer, true)
		character.set_collision_mask_value(cpt_layer, true)
		
		var body_col: Node2D = PhysicsTest2D.get_default_collision_shape(shape_type, 2)
		character.add_child(body_col)
		
		add_child(character)

		var contact_monitor := create_generic_step_monitor(character, test_lambda, physics_step_cbk, simulation_duration)
		contact_monitor.test_name = "%s detects collisions correctly" % [PhysicsTest2D.shape_name(shape_type)]
		
		cpt_layer += 1
