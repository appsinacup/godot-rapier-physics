extends TestBase


func _ready() -> void:
	test_shape_collide_with_distinct_resources()
	test_shape_collide_with_shared_resource()
	print("Shape tests passed.")


func test_shape_collide_with_distinct_resources() -> void:
	var shape := _rectangle()
	var other := _rectangle()
	_run_shape_collide_contract(shape, other)


func test_shape_collide_with_shared_resource() -> void:
	var shape := _rectangle()
	_run_shape_collide_contract(shape, shape)


func _run_shape_collide_contract(shape_a: Shape2D, shape_b: Shape2D) -> void:
	var transform_a := Transform2D(deg_to_rad(20.0), Vector2(25.0, 20.0))
	var overlapping_b := Transform2D(deg_to_rad(-15.0), Vector2(35.0, 24.0))
	var separated_b := Transform2D(deg_to_rad(-15.0), Vector2(200.0, 24.0))

	assert(shape_a.collide(transform_a, shape_b, overlapping_b))
	_assert_contacts(shape_a.collide_and_get_contacts(transform_a, shape_b, overlapping_b), true)
	assert(not shape_a.collide(transform_a, shape_b, separated_b))
	_assert_contacts(shape_a.collide_and_get_contacts(transform_a, shape_b, separated_b), false)

	var start_a := Transform2D(0.0, Vector2.ZERO)
	var start_b := Transform2D(0.0, Vector2(100.0, 0.0))
	assert(shape_a.collide_with_motion(start_a, Vector2(100.0, 0.0), shape_b, start_b, Vector2.ZERO))
	_assert_contacts(
		shape_a.collide_with_motion_and_get_contacts(
			start_a,
			Vector2(100.0, 0.0),
			shape_b,
			start_b,
			Vector2.ZERO,
		),
		true,
	)
	assert(not shape_a.collide_with_motion(start_a, Vector2(-100.0, 0.0), shape_b, start_b, Vector2.ZERO))
	_assert_contacts(
		shape_a.collide_with_motion_and_get_contacts(
			start_a,
			Vector2(-100.0, 0.0),
			shape_b,
			start_b,
			Vector2.ZERO,
		),
		false,
	)


func _rectangle() -> RectangleShape2D:
	var shape := RectangleShape2D.new()
	shape.size = Vector2(40.0, 24.0)
	return shape


func _assert_contacts(contacts: PackedVector2Array, should_collide: bool) -> void:
	if not should_collide:
		assert(contacts.is_empty())
		return
	assert(contacts.size() >= 2)
	assert(contacts.size() % 2 == 0)
	for contact in contacts:
		assert(contact.is_finite())
