extends GutTest

func test_create_circle():
	var shape := PhysicsServer2D.circle_shape_create()
	assert_eq(PhysicsServer2D.shape_get_type(shape), PhysicsServer2D.SHAPE_CIRCLE)
	assert_eq(PhysicsServer2D.shape_get_data(shape), null)
	# float
	PhysicsServer2D.shape_set_data(shape, 3.4)
	print(PhysicsServer2D.shape_get_data(shape))
	assert_almost_eq(PhysicsServer2D.shape_get_data(shape), 3.4, 0.0001)
	# bool
	PhysicsServer2D.shape_set_data(shape, true)
	assert_eq(PhysicsServer2D.shape_get_data(shape), null)
	#PhysicsServer2D.shape_set_data();
	#PhysicsServer2D.free_rid(shape);
