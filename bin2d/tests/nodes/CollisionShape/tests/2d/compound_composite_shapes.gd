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

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != SETTLE_FRAME:
			return

		p_monitor.add_test("World boundary shapes still collide")
		p_monitor.add_test_result(boundary_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.add_test("Concave polygon shapes still collide")
		p_monitor.add_test_result(concave_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.add_test("Skewed multi-shape body still collides")
		p_monitor.add_test_result(skewed_faller.global_position.y < FLOOR_Y + 50.0)

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
