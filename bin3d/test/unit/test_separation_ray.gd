extends TestBase

func _ready():
	test_shape_data_round_trip()
	test_shape_type()
	print("Separation ray tests passed.")

func test_shape_data_round_trip():
	var shape := PhysicsServer3D.separation_ray_shape_create()
	assert(shape != null)
	PhysicsServer3D.shape_set_data(shape, {"length": 2.0, "slide_on_slope": true})
	var data = PhysicsServer3D.shape_get_data(shape)
	assert(data is Dictionary)
	assert_eq(data["length"], 2.0)
	assert(data["slide_on_slope"] == true)
	PhysicsServer3D.shape_set_data(shape, {"length": 0.5, "slide_on_slope": false})
	data = PhysicsServer3D.shape_get_data(shape)
	assert_eq(data["length"], 0.5)
	assert(data["slide_on_slope"] == false)
	PhysicsServer3D.free_rid(shape)

func test_shape_type():
	var shape := PhysicsServer3D.separation_ray_shape_create()
	PhysicsServer3D.shape_set_data(shape, {"length": 1.0, "slide_on_slope": false})
	assert(PhysicsServer3D.shape_get_type(shape) == PhysicsServer3D.SHAPE_SEPARATION_RAY)
	PhysicsServer3D.free_rid(shape)
