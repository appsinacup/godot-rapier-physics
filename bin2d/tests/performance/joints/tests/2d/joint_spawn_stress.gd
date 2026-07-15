extends PhysicsPerformanceTest2D

# LOCAL-ONLY joint stress test (tests/performance is not run by CI). It keeps spawning
# dynamic boxes at a steady rate, each tethered to a single central point by a joint,
# cycling through every 2D joint type. It keeps growing until the smoothed FPS drops below
# MINIMUM_FPS (like the rigid-body perf tests), then reports how many joints it reached and
# holds the final cluster on screen. A high safety cap guarantees termination in headless
# runs where the FPS never actually drops.

const MINIMUM_FPS := 30.0
const PER_SPAWN := 3           # bodies added per spawn tick (the rate of increase)
const SPAWN_EVERY := 0.10
const WARMUP_BODIES := 40      # don't consult FPS until there is a real load
const SAFETY_CAP := 800        # backstop so headless runs terminate
const BOX := Vector2(24, 24)
const HOLD := 4.0              # seconds to keep the final result on screen

var pivot: Vector2
var anchor: StaticBody2D
var timer: Timer
var count := 0
var exploded := false
var bodies: Array[RigidBody2D] = []
var label: Label
var _colors := [Color(0.36, 0.66, 1.0), Color(1.0, 0.6, 0.3), Color(0.5, 0.85, 0.45)]

func test_name() -> String:
	return "Joint2D | STRESS: cluster pinned to a point until FPS < 30 (local only)"

func test_start() -> void:
	super()
	pivot = CENTER
	label = Label.new()
	label.position = TOP_LEFT + Vector2(20, 62)
	label.set("theme_override_font_sizes/font_size", 18)
	add_child(label)

	anchor = StaticBody2D.new()
	anchor.position = pivot
	var acol := CollisionShape2D.new()
	acol.shape = RectangleShape2D.new()
	acol.shape.size = BOX
	anchor.add_child(acol)
	add_child(anchor)

	timer = Timer.new()
	timer.wait_time = SPAWN_EVERY
	timer.process_callback = Timer.TIMER_PROCESS_PHYSICS
	timer.timeout.connect(spawn)
	add_child(timer)
	timer.start()

func _make_body(p_pos: Vector2, p_i: int) -> RigidBody2D:
	var body := RigidBody2D.new()
	body.position = p_pos
	body.linear_damp = 1.0
	body.angular_damp = 1.0
	body.collision_layer = 0
	body.collision_mask = 0
	var col := CollisionShape2D.new()
	col.shape = RectangleShape2D.new()
	col.shape.size = BOX
	body.add_child(col)
	var poly := Polygon2D.new()
	poly.polygon = PackedVector2Array([-BOX * 0.5, Vector2(BOX.x, -BOX.y) * 0.5, BOX * 0.5, Vector2(-BOX.x, BOX.y) * 0.5])
	poly.color = _colors[p_i % _colors.size()]
	body.add_child(poly)
	add_child(body)
	return body

func _add_one() -> void:
	var ang := randf() * TAU
	var dist := 40.0 + randf() * 90.0
	var body := _make_body(pivot + Vector2(cos(ang), sin(ang)) * dist, count)
	bodies.append(body)
	var kind := count % 3
	var joint: Joint2D
	if kind == 0:
		joint = PinJoint2D.new()
	elif kind == 1:
		var spring := DampedSpringJoint2D.new()
		spring.length = dist + 40.0
		spring.rest_length = dist
		spring.stiffness = 120.0
		spring.damping = 4.0
		joint = spring
	else:
		var groove := GrooveJoint2D.new()
		groove.length = dist + 60.0
		joint = groove
	add_child(joint)
	joint.global_position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = body.get_path()
	count += 1

func spawn() -> void:
	for b in bodies:
		if not (is_finite(b.global_position.x) and is_finite(b.global_position.y)) \
				or absf(b.global_position.x) > 100000.0 or absf(b.global_position.y) > 100000.0:
			exploded = true
	label.text = "Joints: %d\n%d FPS" % [count, get_smoothed_fps()]

	var low_fps: bool = count >= WARMUP_BODIES and get_smoothed_fps() <= MINIMUM_FPS
	if low_fps or count >= SAFETY_CAP:
		timer.stop()
		extra_text.append("Reached %d joints pinned to one point before FPS < %d" % [count, MINIMUM_FPS])
		extra_text.append("Cluster stayed finite: %s" % [not exploded])
		register_result("Joints before FPS < %d" % MINIMUM_FPS, "%d (%s)" % [count, "stable" if not exploded else "EXPLODED"])
		test_completed(HOLD)
		return
	for _i in range(PER_SPAWN):
		_add_one()

func clean() -> void:
	pass
