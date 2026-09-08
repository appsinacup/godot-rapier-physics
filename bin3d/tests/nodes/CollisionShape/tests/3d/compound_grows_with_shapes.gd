extends PhysicsUnitTest3D


# A static body gathers its shapes into one compound collider, and the shapes arrive one at a time:
# the server is told about each of them with its own body_add_shape call. Every one of them has to
# end up in the compound, however many arrive after the first.

const SHAPE_COUNT := 7
const SPACING := 2.0
const SETTLE_FRAME := 10

var simulation_duration := 8
var body := RID()
var shapes: Array[RID] = []

func test_description() -> String:
	return """Checks that a static body keeps every shape it is given, rather than only the ones it
	held when its shapes were first gathered into a compound collider.
	"""

func test_name() -> String:
	return "CollisionShape3D | a compound keeps every shape added to it"

func _exit_tree() -> void:
	for shape in shapes:
		PhysicsServer3D.free_rid(shape)
	if body.is_valid():
		PhysicsServer3D.free_rid(body)

func test_start() -> void:
	body = PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_mode(body, PhysicsServer3D.BODY_MODE_STATIC)
	PhysicsServer3D.body_set_space(body, get_world_3d().space)
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D())
	for i in SHAPE_COUNT:
		var shape := PhysicsServer3D.box_shape_create()
		PhysicsServer3D.shape_set_data(shape, Vector3(0.5, 0.5, 0.5))
		shapes.append(shape)
		PhysicsServer3D.body_add_shape(body, shape,
			Transform3D(Basis(), Vector3(i * SPACING, 0.0, 0.0)))

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != SETTLE_FRAME:
			return

		var space := get_world_3d().direct_space_state
		var query := PhysicsShapeQueryParameters3D.new()
		var probe := BoxShape3D.new()
		probe.size = Vector3(0.2, 0.2, 0.2)
		query.shape = probe

		var missing := []
		var reported := {}
		for i in SHAPE_COUNT:
			query.transform = Transform3D(Basis(), Vector3(i * SPACING, 0.0, 0.0))
			var hits := space.intersect_shape(query, 8)
			if hits.is_empty():
				missing.append(i)
			for hit in hits:
				if hit.rid == body:
					reported[hit.shape] = true

		p_monitor.add_test("Every shape added to the body has a collider")
		if not missing.is_empty():
			p_monitor.add_test_error("Shapes %s report no collider." % [missing])
		p_monitor.add_test_result(missing.is_empty())

		p_monitor.add_test("Each shape is reported under its own index")
		if reported.size() != SHAPE_COUNT:
			p_monitor.add_test_error("Reported indices %s, expected %d of them."
				% [reported.keys(), SHAPE_COUNT])
		p_monitor.add_test_result(reported.size() == SHAPE_COUNT)

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
