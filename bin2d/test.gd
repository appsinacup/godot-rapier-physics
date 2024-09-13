extends Node2D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print("Start")
	test_capsule_shape()
	test_circle_shape()
	print("Success")
	get_tree().quit()
	print("Quit")

func test_capsule_shape():
	RapierCapsuleShapeTests.test_allows_one_way_collision()
	RapierCapsuleShapeTests.test_create()
	RapierCapsuleShapeTests.test_get_data()
	RapierCapsuleShapeTests.test_get_type()
	RapierCapsuleShapeTests.test_set_data_from_array()
	RapierCapsuleShapeTests.test_set_data_from_dictionary()
	RapierCapsuleShapeTests.test_set_data_from_vector2()
	
func test_circle_shape():
	RapierCircleShapeTests.test_allows_one_way_collision()
	RapierCircleShapeTests.test_create()
	RapierCircleShapeTests.test_get_data()
	RapierCircleShapeTests.test_get_type()
	RapierCircleShapeTests.test_set_data()
