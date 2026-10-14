extends PhysicsUnitTest2D

var simulation_duration := 10

# Space queries only see a body moved with body_set_state() after the next step.
# space_update_queries() refreshes them in between (#649).

const PROBE_AT := Vector2(200, 0)
const FAR := Vector2(0, 500)
const AREA := -1
const CASES := {
	"a static body": PhysicsServer2D.BODY_MODE_STATIC,
	"a rigid body": PhysicsServer2D.BODY_MODE_RIGID,
	"an area": AREA,
}

var space: RID
var shape: RID

func test_description() -> String:
	return """Checks that space_update_queries() lets space queries see static bodies, rigid
	bodies and areas moved since the last step, without stepping.
	"""

func test_name() -> String:
	return "DirectSpaceState2D | space_update_queries sees moves without a step (#649)"

func test_start() -> void:
	# Its own space, stepped by hand.
	space = PhysicsServer2D.space_create()
	PhysicsServer2D.space_set_active(space, false)
	shape = PhysicsServer2D.rectangle_shape_create()
	PhysicsServer2D.shape_set_data(shape, Vector2(10, 10))

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		for label in CASES:
			var mode: int = CASES[label]
			var object := add_object(mode)
			move(object, mode, FAR)
			step()

			move(object, mode, PROBE_AT)
			RapierPhysicsServer2D.space_update_queries(space)
			p_monitor.add_test("Once queries are updated, %s moved since the last step is found" % label)
			p_monitor.add_test_result(probe() == 1)

			move(object, mode, FAR)
			RapierPhysicsServer2D.space_update_queries(space)
			p_monitor.add_test("Once queries are updated, %s moved away is no longer found" % label)
			p_monitor.add_test_result(probe() == 0)

			PhysicsServer2D.free_rid(object)
		PhysicsServer2D.free_rid(shape)
		PhysicsServer2D.free_rid(space)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func add_object(p_mode: int) -> RID:
	if p_mode == AREA:
		var area := PhysicsServer2D.area_create()
		PhysicsServer2D.area_add_shape(area, shape)
		PhysicsServer2D.area_set_space(area, space)
		return area
	var body := PhysicsServer2D.body_create()
	PhysicsServer2D.body_set_mode(body, p_mode)
	PhysicsServer2D.body_add_shape(body, shape)
	PhysicsServer2D.body_set_space(body, space)
	return body

func move(p_object: RID, p_mode: int, p_position: Vector2) -> void:
	if p_mode == AREA:
		PhysicsServer2D.area_set_transform(p_object, Transform2D(0, p_position))
	else:
		PhysicsServer2D.body_set_state(p_object, PhysicsServer2D.BODY_STATE_TRANSFORM, Transform2D(0, p_position))

func step() -> void:
	RapierPhysicsServer2D.space_step(space, 1.0 / 60)
	RapierPhysicsServer2D.space_flush_queries(space)

# The number of bodies and areas at PROBE_AT.
func probe() -> int:
	var circle := CircleShape2D.new()
	circle.radius = 5
	var query := PhysicsShapeQueryParameters2D.new()
	query.shape = circle
	query.transform = Transform2D(0, PROBE_AT)
	query.collide_with_areas = true
	return PhysicsServer2D.space_get_direct_state(space).intersect_shape(query).size()
