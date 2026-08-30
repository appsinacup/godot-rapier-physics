extends PhysicsUnitTest2D


# A static body with more than one shape puts them all in one compound collider. Parry refuses a
# compound whose parts are themselves composite -- a world boundary, a concave polygon, a heightmap
# -- and panics rather than reporting it, which takes the whole extension down. Such an object has
# to stay on one collider per shape instead. A 2D skew is the same story once removed: it turns any
# shape into a compound while the collider is being built, long after the shape type was checked.

const FLOOR_Y := 500.0
const DROP_HEIGHT := 200.0
const SETTLE_FRAME := 90

var simulation_duration := 10
var boundary_faller: RigidBody2D
var concave_faller: RigidBody2D
var skewed_faller: RigidBody2D
var mixed: StaticBody2D
var indexed: StaticBody2D
var one_way_body: StaticBody2D
var one_way_faller: RigidBody2D
var mixed_box_faller: RigidBody2D
var mixed_concave_faller: RigidBody2D

func test_description() -> String:
	return """Checks that a static body holding several composite shapes, or several shapes under a
	skew, still collides instead of crashing or silently losing its colliders.
	"""

func test_name() -> String:
	return "CollisionShape2D | multi-shape static bodies with composite shapes"

func _add_faller(p_x: float) -> RigidBody2D:
	var body := RigidBody2D.new()
	var shape := CollisionShape2D.new()
	var box := RectangleShape2D.new()
	box.size = Vector2(40, 40)
	shape.shape = box
	body.add_child(shape)
	body.position = Vector2(p_x, FLOOR_Y - DROP_HEIGHT)
	add_child(body)
	return body

func _box(p_size: Vector2, p_pos: Vector2) -> CollisionShape2D:
	var shape := CollisionShape2D.new()
	var box := RectangleShape2D.new()
	box.size = p_size
	shape.shape = box
	shape.position = p_pos
	return shape


func _boundary(p_normal: Vector2, p_distance: float) -> CollisionShape2D:
	var shape := CollisionShape2D.new()
	var boundary := WorldBoundaryShape2D.new()
	boundary.normal = p_normal
	boundary.distance = p_distance
	shape.shape = boundary
	return shape

func _segments(p_from: Vector2, p_to: Vector2) -> CollisionShape2D:
	var shape := CollisionShape2D.new()
	var concave := ConcavePolygonShape2D.new()
	concave.segments = PackedVector2Array([p_from, p_to])
	shape.shape = concave
	return shape

func test_start() -> void:
	# Two world boundaries: each is a compound of one halfspace, so both parts are composite.
	var boundaries := StaticBody2D.new()
	boundaries.add_child(_boundary(Vector2(0, -1), -FLOOR_Y))
	boundaries.add_child(_boundary(Vector2(1, 0), 0.0))
	add_child(boundaries)
	boundary_faller = _add_faller(150.0)

	# Two concave polygons: each is a polyline, also composite.
	var concaves := StaticBody2D.new()
	concaves.add_child(_segments(Vector2(300, FLOOR_Y), Vector2(500, FLOOR_Y)))
	concaves.add_child(_segments(Vector2(500, FLOOR_Y), Vector2(700, FLOOR_Y)))
	add_child(concaves)
	concave_faller = _add_faller(400.0)

	# Two plain rectangles, but skewed: scale_shape turns each into a compound while building.
	var skewed := StaticBody2D.new()
	skewed.position = Vector2(900, FLOOR_Y)
	for offset in [-100.0, 100.0]:
		var shape := CollisionShape2D.new()
		var box := RectangleShape2D.new()
		box.size = Vector2(200, 40)
		shape.shape = box
		shape.position = Vector2(offset, 0)
		shape.skew = 0.5
		skewed.add_child(shape)
	add_child(skewed)
	skewed_faller = _add_faller(900.0)

	# A body mixing shapes that can share a compound with one that cannot: the boxes still gather
	# into a compound, the concave keeps a collider of its own, and both still collide.
	mixed = StaticBody2D.new()
	mixed.add_child(_box(Vector2(200, 40), Vector2(1300, FLOOR_Y)))
	mixed.add_child(_box(Vector2(200, 40), Vector2(1500, FLOOR_Y)))
	mixed.add_child(_segments(Vector2(1700, FLOOR_Y), Vector2(1900, FLOOR_Y)))
	add_child(mixed)
	mixed_box_faller = _add_faller(1400.0)
	mixed_concave_faller = _add_faller(1800.0)

	# Three plain boxes: every one is a compound member, so the compound alone answers for all of
	# them and has to report the same shape index the authored child sits at.
	indexed = StaticBody2D.new()
	# point queries honor pickable by default (physics/rapier/queries/point_query_honors_pickable)
	indexed.input_pickable = true
	for i in 3:
		indexed.add_child(_box(Vector2(80, 40), Vector2(2100 + i * 100, FLOOR_Y)))
	add_child(indexed)

	# A one-way shape among plain ones: it keeps a collider of its own so it stays one-way, while
	# the plain ones still gather into a compound.
	one_way_body = StaticBody2D.new()
	one_way_body.add_child(_box(Vector2(200, 40), Vector2(2600, FLOOR_Y)))
	one_way_body.add_child(_box(Vector2(200, 40), Vector2(2800, FLOOR_Y)))
	var passable := _box(Vector2(200, 40), Vector2(3000, FLOOR_Y))
	passable.one_way_collision = true
	one_way_body.add_child(passable)
	add_child(one_way_body)
	one_way_faller = _add_faller(2700.0)

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != SETTLE_FRAME:
			return

		p_monitor.add_test("World boundary shapes still collide")
		p_monitor.add_test_result(boundary_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.add_test("Concave polygon shapes still collide")
		p_monitor.add_test_result(concave_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.add_test("Skewed multi-shape body still collides")
		p_monitor.add_test_result(skewed_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.add_test("Mixed body still collides on its compound-able shapes")
		p_monitor.add_test_result(mixed_box_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.add_test("Mixed body still collides on the shape left out of the compound")
		p_monitor.add_test_result(mixed_concave_faller.global_position.y < FLOOR_Y + 50.0)

		# The compound answers for its members and the left-out shape for itself. If a hit on the
		# compound expanded to every shape of the object, the left-out one would come back twice.
		var space := get_world_2d().direct_space_state
		var query := PhysicsShapeQueryParameters2D.new()
		var probe := RectangleShape2D.new()
		probe.size = Vector2(700, 20)
		query.shape = probe
		query.transform = Transform2D(0.0, Vector2(1600, FLOOR_Y))
		var seen := {}
		var duplicated := false
		for hit in space.intersect_shape(query, 32):
			if hit.collider != mixed:
				continue
			if seen.has(hit.shape):
				duplicated = true
			seen[hit.shape] = true

		p_monitor.add_test("Mixed body reports each shape once")
		p_monitor.add_test_result(not duplicated)

		p_monitor.add_test("Mixed body reports every one of its shapes")
		p_monitor.add_test_result(seen.size() == 3)

		# Each box must come back as the index of the child that authored it, the way it would if
		# every shape still had a collider of its own.
		p_monitor.add_test("Compound reports the authored shape index")
		var indices_match := true
		for i in 3:
			var point := PhysicsPointQueryParameters2D.new()
			point.position = Vector2(2100 + i * 100, FLOOR_Y)
			var reported := -1
			for hit in space.intersect_point(point, 32):
				if hit.collider == indexed:
					reported = hit.shape
			if reported != i:
				p_monitor.add_test_error("box %d reported as shape %d" % [i, reported])
				indices_match = false
		p_monitor.add_test_result(indices_match)

		p_monitor.add_test("One-way shape stays out of the compound and stays one-way")
		var one_way_shapes := {}
		var one_way_probe := PhysicsShapeQueryParameters2D.new()
		var wide := RectangleShape2D.new()
		wide.size = Vector2(700, 20)
		one_way_probe.shape = wide
		one_way_probe.transform = Transform2D(0.0, Vector2(2800, FLOOR_Y))
		for hit in space.intersect_shape(one_way_probe, 32):
			if hit.collider == one_way_body:
				one_way_shapes[hit.shape] = true
		p_monitor.add_test_result(one_way_shapes.size() == 3)

		p_monitor.add_test("Solid shapes of a one-way body still collide")
		p_monitor.add_test_result(one_way_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
