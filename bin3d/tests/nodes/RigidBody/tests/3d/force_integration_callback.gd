extends PhysicsUnitTest3D

const USER_DATA := "user data"

var simulation_duration := 1.0
var plain_args := []
var user_data_args := []
var call_order := []

func test_description() -> String:
	return """Checks that force integration callbacks receive the body state and user data even
	when the body's direct state was never requested, and that they run before the state sync.
	"""

func test_name() -> String:
	return "RigidBody3D | testing the force integration callback arguments"

func record_plain(...args: Array) -> void:
	plain_args = args

func record_user_data(...args: Array) -> void:
	user_data_args = args

func record_integration(_state) -> void:
	call_order.append("integration")

func record_sync(_state) -> void:
	call_order.append("sync")

func create_body(shape: RID) -> RID:
	var body := PhysicsServer3D.body_create()
	PhysicsServer3D.body_add_shape(body, shape)
	PhysicsServer3D.body_set_collision_layer(body, 0)
	PhysicsServer3D.body_set_collision_mask(body, 0)
	PhysicsServer3D.body_set_space(body, get_world_3d().space)
	return body

func test_start() -> void:
	var shape := PhysicsServer3D.sphere_shape_create()
	PhysicsServer3D.shape_set_data(shape, 10.0)

	var plain := create_body(shape)
	PhysicsServer3D.body_set_force_integration_callback(plain, record_plain)
	var with_user_data := create_body(shape)
	PhysicsServer3D.body_set_force_integration_callback(with_user_data, record_user_data, USER_DATA)
	var ordered := create_body(shape)
	PhysicsServer3D.body_set_force_integration_callback(ordered, record_integration)
	PhysicsServer3D.body_set_state_sync_callback(ordered, record_sync)

	var run = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 5:
			return
		p_monitor.add_test("Callback receives the body state")
		p_monitor.add_test_result(plain_args.size() == 1 and plain_args[0] is PhysicsDirectBodyState3D)
		p_monitor.add_test("Callback receives the user data")
		p_monitor.add_test_result(user_data_args.size() == 2 and user_data_args[1] == USER_DATA)
		p_monitor.add_test("Force integration runs before the state sync")
		p_monitor.add_test_result(call_order.size() >= 2 and call_order.slice(-2) == ["integration", "sync"])
		for body in [plain, with_user_data, ordered]:
			PhysicsServer3D.free_rid(body)
		PhysicsServer3D.free_rid(shape)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
