extends PhysicsUnitTest2D

@onready var collision_shape_1 : CollisionShape2D = $StaticBody2D/CollisionShape2D
@onready var collision_shape_2 : CollisionShape2D = $StaticBody2D2/CollisionShape2D

func test_description() -> String:
	return """Checks if the Shape Disable is working.
	"""
	
func test_name() -> String:
	return "RigidBody2D | testing Shape Disable"

var detect_x_collision := false
var detect_y_collision := false

func test_start() -> void:
	collision_shape_1.disabled = !collision_shape_1.disabled
	collision_shape_2.disabled = !collision_shape_2.disabled
