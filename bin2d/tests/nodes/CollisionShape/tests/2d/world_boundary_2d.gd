extends PhysicsUnitTest2D

func test_description() -> String:
	return """
	"""
	
func test_name() -> String:
	return "CollisionShape2D | testing [One Way Collision] with World Boundary 2D"
	
func test_start() -> void:
	var world_boundary := create_static_body(-25, CENTER)
	world_boundary.position = BOTTOM_CENTER - Vector2(0, 50)
	var body := create_character(1)
	body.position = CENTER

func create_static_body(p_rotation: float, p_force: Vector2) -> StaticBody2D:
	var body := StaticBody2D.new()   
	var world_boundary := WorldBoundaryShape2D.new()
	var collision_shape := CollisionShape2D.new()
	collision_shape.shape = world_boundary
	collision_shape.rotation = deg_to_rad(p_rotation)
	body.name = "world_boundary"
	body.add_child(collision_shape)
	add_child(body)
	return body

func create_character(p_layer: int, p_body_shape := PhysicsTest2D.TestCollisionShape.RECTANGLE) -> CharacterBody2D:
	var character = CharacterBody2D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd")
	#character.collision_layer = 0
	#character.collision_mask = 0
	character.set_collision_layer_value(p_layer, true)
	character.set_collision_mask_value(p_layer, true)
	var body_col: Node2D = get_default_collision_shape(p_body_shape, 2)
	character.add_child(body_col)
	add_child(character)
	return character
