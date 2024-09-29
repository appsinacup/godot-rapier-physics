extends Node

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print("Start")
	test_capsule_shape()
	test_circle_shape()
	test_concave_polygon_shape()
	test_convex_polygon_shape()
	test_cylinder_shape()
	print("Success")
	await get_tree().create_timer(1.0).timeout
	get_tree().quit()

func test_capsule_shape():
	RapierCapsuleShapeTests.test_create()
	RapierCapsuleShapeTests.test_set_data_from_array()
	RapierCapsuleShapeTests.test_set_data_from_dictionary()
	RapierCapsuleShapeTests.test_set_data_from_vector2()

func test_circle_shape():
	RapierCircleShapeTests.test_create()
	RapierCircleShapeTests.test_set_data()

func test_concave_polygon_shape():
	RapierConcavePolygonShapeTests.test_create()
	RapierConcavePolygonShapeTests.test_set_data()

func test_convex_polygon_shape():  # New function for convex polygon shape
	RapierConvexPolygonShapeTests.test_create()
	RapierConvexPolygonShapeTests.test_set_data()

func test_cylinder_shape():
	RapierCylinderShape3DTests.test_create()
	RapierCylinderShape3DTests.test_set_data_array()
	RapierCylinderShape3DTests.test_set_data_vector2()
	RapierCylinderShape3DTests.test_set_data_dictionary()
