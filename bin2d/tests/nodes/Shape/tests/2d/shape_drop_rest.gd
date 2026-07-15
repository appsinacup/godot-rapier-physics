extends PhysicsUnitTest2D

# Per-shape drop-and-rest test: one dynamic body of each collision shape type is dropped
# onto a static floor. Each shape must collide with the floor (not tunnel through), come to
# rest on top of it, and never explode. This exercises collision + resting for every 2D
# shape the plugin supports.

var simulation_duration := 12.0
const DROP_FRAME := 300
const FLOOR_TOP := 260.0     # world-space y of the floor's top surface

var shapes := [
	PhysicsTest2D.TestCollisionShape.CIRCLE,
	PhysicsTest2D.TestCollisionShape.RECTANGLE,
	PhysicsTest2D.TestCollisionShape.CAPSULE,
	PhysicsTest2D.TestCollisionShape.CONVEX_POLYGON,
]
var bodies: Array[RigidBody2D] = []

func test_description() -> String:
	return "Drops a body of each 2D shape type onto a floor; each must collide, rest on top, and not explode."

func test_name() -> String:
	return "Shape2D | drop and rest (circle/rectangle/capsule/convex)"

func test_start() -> void:
	# Wide static floor whose top surface sits at FLOOR_TOP.
	var floor_body := StaticBody2D.new()
	var floor_col := CollisionShape2D.new()
	floor_col.shape = RectangleShape2D.new()
	floor_col.shape.size = Vector2(Global.WINDOW_SIZE.x, 60)
	floor_col.position = Vector2(Global.WINDOW_SIZE.x * 0.5, FLOOR_TOP + 30)
	floor_body.add_child(floor_col)
	add_child(floor_body)

	var step := Global.WINDOW_SIZE.x / (shapes.size() + 1)
	for i in range(shapes.size()):
		var body := RigidBody2D.new()
		body.position = Vector2(step * (i + 1), FLOOR_TOP - 140)
		body.add_child(PhysicsTest2D.get_default_collision_shape(shapes[i], 2.0))
		add_child(body)
		bodies.append(body)

	var state := {"finite": true}
	var checks := func(_t, m: GenericManualMonitor):
		for b in bodies:
			if not (is_finite(b.global_position.x) and is_finite(b.global_position.y)):
				state.finite = false
		if m.frame >= DROP_FRAME:
			m.add_test("No NaN/Inf in any dropped shape")
			m.add_test_result(state.finite)

			for i in range(bodies.size()):
				var b := bodies[i]
				var name: String = PhysicsTest2D.shape_name(shapes[i])
				var y := b.global_position.y
				var rested: bool = b.linear_velocity.length() < 12.0 and y < FLOOR_TOP and y > FLOOR_TOP - 90.0
				m.add_test("%s rests on the floor (no tunneling)" % name)
				if not rested:
					m.add_test_error("y=%.1f (floor top %.0f) v=%.2f" % [y, FLOOR_TOP, b.linear_velocity.length()])
				m.add_test_result(rested)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
