extends PhysicsPerformanceTest2D

# LOCAL-ONLY joint stress test (tests/performance is not run by CI). It builds a MATRIX:
# a top row of static anchors, then keeps adding levels below, each new body pinned to the
# body above it AND to its left neighbour, forming a growing hanging net. It keeps adding
# levels until the smoothed FPS drops below MINIMUM_FPS (like the rigid-body perf tests),
# then reports the size it reached and holds the net on screen. A safety cap guarantees
# termination in headless runs where the FPS never actually drops.

const COLS := 8
const SPACING := 42.0
const MINIMUM_FPS := 30.0
const SPAWN_EVERY := 0.2
const WARMUP_ROWS := 4
const SAFETY_ROWS := 60        # backstop so headless runs terminate
const BOX := Vector2(20, 20)
const HOLD := 4.0

var timer: Timer
var rows := 0
var prev_row: Array[RigidBody2D] = []
var anchors: Array[StaticBody2D] = []
var all_bodies: Array[RigidBody2D] = []
var origin: Vector2
var exploded := false
var label: Label

func test_name() -> String:
	return "Joint2D | STRESS: pinned matrix / net until FPS < 30 (local only)"

func test_start() -> void:
	super()
	label = Label.new()
	label.position = TOP_LEFT + Vector2(20, 62)
	label.set("theme_override_font_sizes/font_size", 18)
	add_child(label)

	origin = Vector2(Global.WINDOW_SIZE.x * 0.5 - (COLS - 1) * SPACING * 0.5, 80.0)
	for c in range(COLS):
		var a := StaticBody2D.new()
		a.position = origin + Vector2(c * SPACING, 0)
		var col := CollisionShape2D.new()
		col.shape = RectangleShape2D.new()
		col.shape.size = BOX
		a.add_child(col)
		add_child(a)
		anchors.append(a)

	timer = Timer.new()
	timer.wait_time = SPAWN_EVERY
	timer.process_callback = Timer.TIMER_PROCESS_PHYSICS
	timer.timeout.connect(add_level)
	add_child(timer)
	timer.start()

func _make_body(p_pos: Vector2) -> RigidBody2D:
	var body := RigidBody2D.new()
	body.position = p_pos
	body.linear_damp = 1.2
	body.angular_damp = 1.2
	body.collision_layer = 0
	body.collision_mask = 0
	var col := CollisionShape2D.new()
	col.shape = RectangleShape2D.new()
	col.shape.size = BOX
	body.add_child(col)
	var poly := Polygon2D.new()
	poly.polygon = PackedVector2Array([-BOX * 0.5, Vector2(BOX.x, -BOX.y) * 0.5, BOX * 0.5, Vector2(-BOX.x, BOX.y) * 0.5])
	poly.color = Color(0.4, 0.7, 1.0)
	body.add_child(poly)
	add_child(body)
	return body

func _pin(a: PhysicsBody2D, b: PhysicsBody2D) -> void:
	var j := PinJoint2D.new()
	add_child(j)
	j.global_position = (a.global_position + b.global_position) * 0.5
	j.node_a = a.get_path()
	j.node_b = b.get_path()

func add_level() -> void:
	for b in all_bodies:
		if not (is_finite(b.global_position.x) and is_finite(b.global_position.y)) or b.global_position.length() > 100000.0:
			exploded = true
	label.text = "Net: %d x %d (%d bodies)\n%d FPS" % [rows, COLS, all_bodies.size(), get_smoothed_fps()]

	var low_fps: bool = rows >= WARMUP_ROWS and get_smoothed_fps() <= MINIMUM_FPS
	if low_fps or rows >= SAFETY_ROWS:
		timer.stop()
		extra_text.append("Reached a %d x %d pinned matrix (%d bodies) before FPS < %d" % [rows, COLS, all_bodies.size(), MINIMUM_FPS])
		extra_text.append("Net stayed finite: %s" % [not exploded])
		register_result("Matrix before FPS < %d" % MINIMUM_FPS, "%dx%d (%s)" % [rows, COLS, "stable" if not exploded else "EXPLODED"])
		test_completed(HOLD)
		return
	var above: Array = prev_row if not prev_row.is_empty() else anchors
	var new_row: Array[RigidBody2D] = []
	var y := origin.y + (rows + 1) * SPACING
	for c in range(COLS):
		var body := _make_body(Vector2(origin.x + c * SPACING, y))
		new_row.append(body)
		all_bodies.append(body)
		_pin(above[c], body)
		if c > 0:
			_pin(new_row[c - 1], body)
	prev_row = new_row
	rows += 1

func clean() -> void:
	pass
