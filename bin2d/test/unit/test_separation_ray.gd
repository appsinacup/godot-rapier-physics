extends TestBase

func _ready():
	test_shape_data_round_trip()
	test_shape_type()
	print("Separation ray tests passed.")

func test_shape_data_round_trip():
	var shape := PhysicsServer2D.separation_ray_shape_create()
	assert(shape != null)
	PhysicsServer2D.shape_set_data(shape, {"length": 32.0, "slide_on_slope": true})
	var data = PhysicsServer2D.shape_get_data(shape)
	assert(data is Dictionary)
	assert_eq(data["length"], 32.0)
	assert(data["slide_on_slope"] == true)
	PhysicsServer2D.shape_set_data(shape, {"length": 8.0, "slide_on_slope": false})
	data = PhysicsServer2D.shape_get_data(shape)
	assert_eq(data["length"], 8.0)
	assert(data["slide_on_slope"] == false)
	PhysicsServer2D.free_rid(shape)

func test_shape_type():
	var shape := PhysicsServer2D.separation_ray_shape_create()
	PhysicsServer2D.shape_set_data(shape, {"length": 20.0, "slide_on_slope": false})
	assert(PhysicsServer2D.shape_get_type(shape) == PhysicsServer2D.SHAPE_SEPARATION_RAY)
	PhysicsServer2D.free_rid(shape)
