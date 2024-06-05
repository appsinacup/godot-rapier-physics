extends TestBase

func _ready():
	test_space_functions()
	print("Space tests passed.")


# Test functions for space functions
func test_space_functions():
	var space_rid = RID()
	
	var created_space_rid = PhysicsServer2D.space_create()
	assert(created_space_rid != null)

	var direct_state = PhysicsServer2D.space_get_direct_state(space_rid)
	assert(direct_state == null)

	var space_param = PhysicsServer2D.space_get_param(space_rid, PhysicsServer2D.SpaceParameter.SPACE_PARAM_SOLVER_ITERATIONS)
	assert(space_param == 0.0)

	var is_active = PhysicsServer2D.space_is_active(space_rid)
	assert(is_active == false)

	PhysicsServer2D.space_set_active(space_rid, true)
	PhysicsServer2D.space_set_param(space_rid, PhysicsServer2D.SpaceParameter.SPACE_PARAM_SOLVER_ITERATIONS, 0.0)
