extends PhysicsPerformanceTest3D

# LOCAL-ONLY joint stress test (tests/performance is not run by CI). It keeps spawning
# dynamic boxes at a steady rate, each tethered to a single central point by a joint,
# cycling through every 3D joint type. It keeps growing until the smoothed FPS drops below
# MINIMUM_FPS (like the rigid-body perf tests), then reports how many joints it reached and
# holds the final cluster on screen. A safety cap guarantees termination in headless runs.

const MINIMUM_FPS := 30.0
const PER_SPAWN := 3
const SPAWN_EVERY := 0.10
const WARMUP_BODIES := 40
const SAFETY_CAP := 700
const BOX := Vector3(0.5, 0.5, 0.5)
const HOLD := 4.0

var pivot := Vector3.ZERO
var anchor: StaticBody3D
var timer: Timer
var count := 0
var exploded := false
var bodies: Array[RigidBody3D] = []
var label: Label
var _colors := [Color(0.36, 0.66, 1.0), Color(1.0, 0.6, 0.3), Color(0.5, 0.85, 0.45)]

func test_name() -> String:
	return "Joint3D | STRESS: cluster pinned to a point until FPS < 30 (local only)"

func test_start() -> void:
	super()
	var cam := Camera3D.new()
	cam.position = Vector3(0, 0, 14)
	add_child(cam)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -35, 0)
	add_child(light)
	label = Label.new()
	label.position = Vector2(20, 62)
	label.set("theme_override_font_sizes/font_size", 18)
	add_child(label)

	anchor = StaticBody3D.new()
	anchor.position = pivot
	var acol := CollisionShape3D.new()
	acol.shape = BoxShape3D.new()
	acol.shape.size = BOX
	anchor.add_child(acol)
	add_child(anchor)

	timer = Timer.new()
	timer.wait_time = SPAWN_EVERY
	timer.process_callback = Timer.TIMER_PROCESS_PHYSICS
	timer.timeout.connect(spawn)
	add_child(timer)
	timer.start()

func _make_body(p_pos: Vector3, p_i: int) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.position = p_pos
	body.linear_damp = 1.0
	body.angular_damp = 1.0
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
	mat.albedo_color = _colors[p_i % _colors.size()]
	bm.material = mat
	mesh.mesh = bm
	body.add_child(mesh)
	add_child(body)
	return body

func _lock_linear(joint: Generic6DOFJoint3D) -> void:
	for axis in ["x", "y", "z"]:
		joint.set("linear_limit_%s/enabled" % axis, true)
		joint.set("linear_limit_%s/upper_distance" % axis, 0.0)
		joint.set("linear_limit_%s/lower_distance" % axis, 0.0)

func _add_one() -> void:
	var dir := Vector3(randf() - 0.5, randf() - 0.5, randf() - 0.5).normalized()
	var dist := 1.5 + randf() * 2.5
	var body := _make_body(pivot + dir * dist, count)
	bodies.append(body)
	var kind := count % 5
	var joint: Joint3D
	if kind == 0:
		joint = PinJoint3D.new()
	elif kind == 1:
		joint = HingeJoint3D.new()
	elif kind == 2:
		var slider := SliderJoint3D.new()
		slider.set("linear_limit/upper_distance", 4.0)
		slider.set("linear_limit/lower_distance", -4.0)
		joint = slider
	elif kind == 3:
		var cone := ConeTwistJoint3D.new()
		cone.set("swing_span", deg_to_rad(40.0))
		cone.set("twist_span", deg_to_rad(30.0))
		joint = cone
	else:
		var g6 := Generic6DOFJoint3D.new()
		_lock_linear(g6)
		joint = g6
	add_child(joint)
	joint.global_position = pivot
	joint.node_a = anchor.get_path()
	joint.node_b = body.get_path()
	count += 1

func spawn() -> void:
	for b in bodies:
		if not (is_finite(b.global_position.x) and is_finite(b.global_position.y) and is_finite(b.global_position.z)) \
				or b.global_position.length() > 100000.0:
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
