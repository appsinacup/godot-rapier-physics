extends PhysicsUnitTest3D

var simulation_duration := 10

# A kinematic body only moves at the step, as in Godot. space_update_queries() doesn't change that
# (#649).

const PROBE_AT := Vector3(200, 0, 0)
const FAR := Vector3(0, 500, 0)

var space: RID
var shape: RID

func test_description() -> String:
	return """Checks that after space_update_queries(), space queries still see a kinematic body
	where it was until the step.
	"""

func test_name() -> String:
	return "DirectSpaceState3D | space_update_queries leaves kinematic bodies to the step (#649)"

func test_start() -> void:
	# Its own space, stepped by hand.
	space = PhysicsServer3D.space_create()
	PhysicsServer3D.space_set_active(space, false)
	shape = PhysicsServer3D.box_shape_create()
	PhysicsServer3D.shape_set_data(shape, Vector3(10, 10, 10))

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		var kinematic := PhysicsServer3D.body_create()
		PhysicsServer3D.body_set_mode(kinematic, PhysicsServer3D.BODY_MODE_KINEMATIC)
		PhysicsServer3D.body_add_shape(kinematic, shape)
		PhysicsServer3D.body_set_space(kinematic, space)
		PhysicsServer3D.body_set_state(kinematic, PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(), FAR))
		step()

		PhysicsServer3D.body_set_state(kinematic, PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(), PROBE_AT))
		RapierPhysicsServer3D.space_update_queries(space)
		p_monitor.add_test("A kinematic body stays at its old place until the step")
		p_monitor.add_test_result(probe() == 0)

		step()
		p_monitor.add_test("A kinematic body is found at its new place after the step")
		p_monitor.add_test_result(probe() == 1)

		PhysicsServer3D.free_rid(kinematic)
		PhysicsServer3D.free_rid(shape)
		PhysicsServer3D.free_rid(space)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

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
