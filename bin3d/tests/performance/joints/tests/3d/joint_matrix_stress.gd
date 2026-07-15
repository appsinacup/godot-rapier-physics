extends PhysicsPerformanceTest3D

# LOCAL-ONLY joint stress test (tests/performance is not run by CI). It builds a MATRIX:
# a top row of static anchors, then keeps adding levels below, each new body pinned to the
# body above it AND to its left neighbour, forming a growing hanging net. It keeps adding
# levels until the smoothed FPS drops below MINIMUM_FPS (like the rigid-body perf tests),
# then reports the size it reached and holds the net on screen. A safety cap guarantees
# termination in headless runs where the FPS never actually drops.

const COLS := 12
const SPACING := 1.0
const MINIMUM_FPS := 30.0
const SPAWN_EVERY := 0.2
const WARMUP_ROWS := 4
const SAFETY_ROWS := 50
const BOX := Vector3(0.4, 0.4, 0.4)
const HOLD := 4.0

var timer: Timer
var rows := 0
var prev_row: Array[RigidBody3D] = []
var anchors: Array[StaticBody3D] = []
var all_bodies: Array[RigidBody3D] = []
var origin: Vector3
var exploded := false
var label: Label

func test_name() -> String:
	return "Joint3D | STRESS: pinned matrix / net until FPS < 30 (local only)"

func test_start() -> void:
	super()
	var cam := Camera3D.new()
	cam.position = Vector3(0, -3, 18)
	cam.look_at_from_position(cam.position, Vector3(0, -3, 0), Vector3.UP)
	add_child(cam)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	add_child(light)
	label = Label.new()
	label.position = Vector2(20, 62)
	label.set("theme_override_font_sizes/font_size", 18)
	add_child(label)

	origin = Vector3(-(COLS - 1) * SPACING * 0.5, 2.5, 0)
	for c in range(COLS):
		var a := StaticBody3D.new()
		a.position = origin + Vector3(c * SPACING, 0, 0)
		var col := CollisionShape3D.new()
		col.shape = BoxShape3D.new()
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

func _make_body(p_pos: Vector3) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.position = p_pos
	body.linear_damp = 1.2
	body.angular_damp = 1.2
	body.collision_layer = 0
	body.collision_mask = 0
	var col := CollisionShape3D.new()
	col.shape = BoxShape3D.new()
	col.shape.size = BOX
	body.add_child(col)
	var mesh := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = BOX
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.4, 0.7, 1.0)
	bm.material = mat
	mesh.mesh = bm
	body.add_child(mesh)
	add_child(body)
	return body

func _pin(a: PhysicsBody3D, b: PhysicsBody3D) -> void:
	var j := PinJoint3D.new()
	add_child(j)
	j.global_position = (a.global_position + b.global_position) * 0.5
	j.node_a = a.get_path()
	j.node_b = b.get_path()

func add_level() -> void:
	for b in all_bodies:
		if not (is_finite(b.global_position.x) and is_finite(b.global_position.y) and is_finite(b.global_position.z)) \
				or b.global_position.length() > 100000.0:
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
	var new_row: Array[RigidBody3D] = []
	var y := origin.y - (rows + 1) * SPACING
	for c in range(COLS):
		var body := _make_body(Vector3(origin.x + c * SPACING, y, 0))
		new_row.append(body)
		all_bodies.append(body)
		_pin(above[c], body)
		if c > 0:
			_pin(new_row[c - 1], body)
	prev_row = new_row
	rows += 1

func clean() -> void:
	pass
