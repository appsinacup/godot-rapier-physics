extends TestBase

func _ready():
	test_space_functions()
	print("Space tests passed.")


func test_space_functions():
	var space := PhysicsServer3D.space_create()
	assert(space != null)
	var direct_state = PhysicsServer3D.space_get_direct_state(space)
	assert(direct_state != null)
	var is_active = PhysicsServer3D.space_is_active(space)
	assert(is_active == false)
	PhysicsServer3D.space_set_active(space, true)
	is_active = PhysicsServer3D.space_is_active(space)
	assert(is_active == true)
	PhysicsServer3D.free_rid(space)
	direct_state = PhysicsServer3D.space_get_direct_state(space)
	assert(direct_state == null)
