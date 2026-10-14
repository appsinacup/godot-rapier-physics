extends PhysicsUnitTest2D

var simulation_duration := 10

# A kinematic body only moves at the step, as in Godot. space_update_queries() doesn't change that
# (#649).

const PROBE_AT := Vector2(200, 0)
const FAR := Vector2(0, 500)

var space: RID
var shape: RID

func test_description() -> String:
	return """Checks that after space_update_queries(), space queries still see a kinematic body
	where it was until the step.
	"""

func test_name() -> String:
	return "DirectSpaceState2D | space_update_queries leaves kinematic bodies to the step (#649)"

func test_start() -> void:
	# Its own space, stepped by hand.
	space = PhysicsServer2D.space_create()
	PhysicsServer2D.space_set_active(space, false)
	shape = PhysicsServer2D.rectangle_shape_create()
	PhysicsServer2D.shape_set_data(shape, Vector2(10, 10))

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		var kinematic := PhysicsServer2D.body_create()
		PhysicsServer2D.body_set_mode(kinematic, PhysicsServer2D.BODY_MODE_KINEMATIC)
		PhysicsServer2D.body_add_shape(kinematic, shape)
		PhysicsServer2D.body_set_space(kinematic, space)
		PhysicsServer2D.body_set_state(kinematic, PhysicsServer2D.BODY_STATE_TRANSFORM, Transform2D(0, FAR))
		step()

		PhysicsServer2D.body_set_state(kinematic, PhysicsServer2D.BODY_STATE_TRANSFORM, Transform2D(0, PROBE_AT))
		RapierPhysicsServer2D.space_update_queries(space)
		p_monitor.add_test("A kinematic body stays at its old place until the step")
		p_monitor.add_test_result(probe() == 0)

		step()
		p_monitor.add_test("A kinematic body is found at its new place after the step")
		p_monitor.add_test_result(probe() == 1)

		PhysicsServer2D.free_rid(kinematic)
		PhysicsServer2D.free_rid(shape)
		PhysicsServer2D.free_rid(space)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func step() -> void:
	RapierPhysicsServer2D.space_step(space, 1.0 / 60)
	RapierPhysicsServer2D.space_flush_queries(space)

# The number of bodies at PROBE_AT.
func probe() -> int:
	var circle := CircleShape2D.new()
	circle.radius = 5
	var query := PhysicsShapeQueryParameters2D.new()
	query.shape = circle
	query.transform = Transform2D(0, PROBE_AT)
	return PhysicsServer2D.space_get_direct_state(space).intersect_shape(query).size()
