extends PhysicsUnitTest2D

const BODY_RADIUS := 15.0
const QUERY_RADIUS := 4.0
const TOUCHING_DISTANCE := BODY_RADIUS + QUERY_RADIUS

var simulation_duration := 10

func test_description() -> String:
	return """Checks that [collide_shape] reports shapes that are exactly touching. The solver
	settles bodies touching rather than overlapping, so a query that only accepts strict
	penetration reports nothing.
	"""

func test_name() -> String:
	return "DirectSpaceState2D | testing [collide_shape] against a touching shape"

func test_start() -> void:
	add_child(build_body(CENTER))

	var d_space := get_world_2d().direct_space_state
	var shape_rid := PhysicsServer2D.circle_shape_create()
	PhysicsServer2D.shape_set_data(shape_rid, QUERY_RADIUS)

	var checks := func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return

		p_monitor.add_test("Reports a shape that is exactly touching")
		p_monitor.add_test_result(collides_at(d_space, shape_rid, TOUCHING_DISTANCE))

		p_monitor.add_test("Reports an overlapping shape")
		p_monitor.add_test_result(collides_at(d_space, shape_rid, TOUCHING_DISTANCE - 1.0))

		p_monitor.add_test("Reports a shape inside the query margin")
		p_monitor.add_test_result(collides_at(d_space, shape_rid, TOUCHING_DISTANCE + 0.5, 1.0))

		p_monitor.add_test("Does not report a separated shape")
		p_monitor.add_test_result(not collides_at(d_space, shape_rid, TOUCHING_DISTANCE + 1.0))

		PhysicsServer2D.free_rid(shape_rid)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func collides_at(
	p_space: PhysicsDirectSpaceState2D, p_shape_rid: RID, p_distance: float, p_margin := 0.0
) -> bool:
	var query := PhysicsShapeQueryParameters2D.new()
	query.shape_rid = p_shape_rid
	query.transform = Transform2D(0, CENTER + Vector2(p_distance, 0))
	query.collide_with_bodies = true
	query.margin = p_margin
	return not p_space.collide_shape(query).is_empty()

func build_body(p_position: Vector2) -> StaticBody2D:
	var body := StaticBody2D.new()
	body.position = p_position
	var col := CollisionShape2D.new()
	var shape := CircleShape2D.new()
	shape.radius = BODY_RADIUS
	col.shape = shape
	body.add_child(col)
	return body
