extends PhysicsUnitTest2D

@export var body_shape: PhysicsTest2D.TestCollisionShape = TestCollisionShape.CAPSULE

var spawn_position = Vector2(150, 400)
var speed = 750

func test_description() -> String:
	return """Tests whether the body is firmly attached to the ground when a [floor_snap_length] is applied,
	and only when the velocity points downwards.
	"""

func test_name() -> String:
	return "CharacterBody2D | testing [floor floor_snap_length] [shape: %s]" % [PhysicsTest2D.shape_name(body_shape)]
	
func test_start() -> void:
	# C1 start in air, touch the ground until he reach the wall
	var character1 := create_character(1) # checks behaviour without snap
	character1.position = spawn_position
	add_child(character1)
	
	var c1_test_lambda = func(p_step: int, p_target: CharacterBody2D, _p_monitor: GenericStepMonitor):
		if p_step == 0: return not p_target.is_on_floor()
		elif p_step == 1: return p_target.is_on_floor_only()
		elif p_step == 2: return p_target.is_on_wall()
		
	var physics_step_cbk = func(p_step: int, p_target: CharacterBody2D, is_transition: bool, _p_monitor: GenericStepMonitor):
		if is_transition and p_step == 1:
			p_target.velocity.x = speed
		
	var c1_monitor := create_generic_step_monitor(character1, c1_test_lambda, physics_step_cbk)
	c1_monitor.test_name = "Snapping works as expected (stick to the floor)"

	# C2 without snap it should be not stick to the ground
	var character2 := create_character(2) # checks behaviour with snap
	character2.position = spawn_position
	character2.floor_snap_length = 0 # turn off snapping ton confirm different behavior
	add_child(character2)
	
	var c2_test_lambda = func(p_step: int, p_target: CharacterBody2D, _p_monitor: GenericStepMonitor):
		if p_step == 0: return not p_target.is_on_floor()
		elif p_step == 1: return p_target.is_on_floor_only()
		elif p_step == 2: return not p_target.is_on_wall()
		elif p_step == 3: return p_target.is_on_wall()

	var c2_monitor := create_generic_step_monitor(character2, c2_test_lambda, physics_step_cbk)
	c2_monitor.test_name = "Snapping has a different behaviour than without it"
	
	# C3 try to jump with
	var character3 := create_character(3) # checks if the body can jump
	character3.position = spawn_position
	add_child(character3)
	
	var c3_test_lambda = func(p_step: int, p_target: CharacterBody2D, _p_monitor: GenericStepMonitor):
		if p_step == 0: return not p_target.is_on_floor()
		elif p_step == 1: return p_target.is_on_floor_only()
		elif p_step == 2: return not p_target.is_on_floor()
		elif p_step == 3: return p_target.is_on_floor()

	var cbk_jump_lambda = func(step: int, p_target: CharacterBody2D, p_is_transition: bool, _p_monitor: GenericStepMonitor):
		if p_is_transition and step == 1:
			p_target.velocity.y = -500
	
	var c3_monitor := create_generic_step_monitor(character3, c3_test_lambda, cbk_jump_lambda)
	c3_monitor.test_name = "Can jump when snapping is applied"

func create_character(layer: int) -> CharacterBody2D:
	var character = CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd")
	character.floor_snap_length = 10
	character.position = Vector2(-350, 50)
	character.collision_layer = 0
	character.collision_mask = 0
	character.set_collision_layer_value(layer, true)
	character.set_collision_mask_value(layer, true)
	var body_col: Node2D = PhysicsTest2D.get_default_collision_shape(body_shape, 2)
	character.add_child(body_col)
	return character
