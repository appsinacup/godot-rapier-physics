extends Node2D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var file = FileAccess.open("user://space.json", FileAccess.WRITE)
	var space_state = RapierPhysicsServer2D.space_export_json(get_viewport().world_2d.space)
	file.store_string(space_state)
	RapierCapsuleShapeTests.test_allows_one_way_collision()
	RapierCapsuleShapeTests.test_create()
	RapierCapsuleShapeTests.test_get_data()
	RapierCapsuleShapeTests.test_get_type()
	RapierCapsuleShapeTests.test_set_data_from_array()
	RapierCapsuleShapeTests.test_set_data_from_dictionary()
	RapierCapsuleShapeTests.test_set_data_from_vector2()
	
	RapierCircleShapeTests.test_allows_one_way_collision()
	RapierCircleShapeTests.test_create()
	RapierCircleShapeTests.test_get_data()
	RapierCircleShapeTests.test_get_type()
	RapierCircleShapeTests.test_set_data()
