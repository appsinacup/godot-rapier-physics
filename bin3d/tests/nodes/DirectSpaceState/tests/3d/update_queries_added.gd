extends PhysicsUnitTest3D

var simulation_duration := 10

# A body added since the last step, or a shape moved within its body, is only seen by space queries
# after the next step. space_update_queries() refreshes them in between (#649).

const PROBE_AT := Vector3(200, 0, 0)
const FAR := Vector3(0, 500, 0)

var space: RID
var shape: RID

func test_description() -> String:
	return """Checks that space_update_queries() lets space queries see bodies added and shapes
	moved within their body since the last step, without stepping.
	"""

func test_name() -> String:
	return "DirectSpaceState3D | space_update_queries sees added bodies and moved shapes (#649)"

func test_start() -> void:
	# Its own space, stepped by hand.
	space = PhysicsServer3D.space_create()
	PhysicsServer3D.space_set_active(space, false)
	shape = PhysicsServer3D.box_shape_create()
	PhysicsServer3D.shape_set_data(shape, Vector3(10, 10, 10))

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		var added := add_static_body(1, PROBE_AT)
		RapierPhysicsServer3D.space_update_queries(space)
		p_monitor.add_test("A body added since the last step is found")
		p_monitor.add_test_result(probe() == 1)
		PhysicsServer3D.free_rid(added)
		step()

		# Two shapes on a static body share one compound collider.
		for shape_count in [1, 2]:
			var body := add_static_body(shape_count, FAR)
			step()
			PhysicsServer3D.body_set_shape_transform(body, shape_count - 1, Transform3D(Basis(), PROBE_AT - FAR))
			RapierPhysicsServer3D.space_update_queries(space)
			p_monitor.add_test("A shape moved within its body is found" if shape_count == 1 else "A shape moved within a compound body is found")
			p_monitor.add_test_result(probe() == 1)
			PhysicsServer3D.free_rid(body)
			step()
		PhysicsServer3D.free_rid(shape)
		PhysicsServer3D.free_rid(space)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func add_static_body(p_shape_count: int, p_position: Vector3) -> RID:
	var body := PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_mode(body, PhysicsServer3D.BODY_MODE_STATIC)
	for i in p_shape_count:
		PhysicsServer3D.body_add_shape(body, shape)
	PhysicsServer3D.body_set_space(body, space)
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(), p_position))
	return body

func step() -> void:
	RapierPhysicsServer3D.space_step(space, 1.0 / 60)
	RapierPhysicsServer3D.space_flush_queries(space)

# The number of bodies at PROBE_AT.
func probe() -> int:
	var sphere := SphereShape3D.new()
	sphere.radius = 5
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = sphere
	query.transform = Transform3D(Basis(), PROBE_AT)
	return PhysicsServer3D.space_get_direct_state(space).intersect_shape(query).size()
