extends Node2D
## Cross-platform determinism test.
##
## Runs a reproducible physics simulation for a fixed number of frames,
## records the exact state of every body each frame using bit-exact float
## representation, and writes the result to a JSON file.
##
## The CI pipeline runs this on multiple OS/architecture combinations and
## compares the output files byte-for-byte.

const SIMULATION_FRAMES := 300
const OUTPUT_PATH := "user://determinism_output.json"

var bodies: Array[RigidBody2D] = []
var joints: Array[Joint2D] = []
var frame_records: Array[Dictionary] = []
var current_frame := 0
var simulation_started := false

func _ready() -> void:
	# Use fixed physics settings to ensure reproducibility
	Engine.physics_ticks_per_second = 60
	Engine.max_physics_steps_per_frame = 1

	_setup_simulation()
	simulation_started = true

func _physics_process(_delta: float) -> void:
	if not simulation_started:
		return

	current_frame += 1

	# Record state every frame
	var frame_state := {}
	for i in bodies.size():
		var body := bodies[i]
		frame_state["body_%d" % i] = _serialize_body(body)
	frame_records.append(frame_state)

	if current_frame >= SIMULATION_FRAMES:
		_finish()

func _setup_simulation() -> void:
	# Create world boundaries (floor and walls)
	_add_boundaries()

	# Scenario 1: Stack of boxes (tests resting contact + friction)
	_create_box_stack()

	# Scenario 2: Bodies with applied forces and torques
	_create_force_bodies()

	# Scenario 3: Chain of bodies connected by pin joints
	_create_joint_chain()

	# Scenario 4: Different shapes interacting (circle vs convex vs rectangle)
	_create_mixed_shapes()

	# Scenario 5: High-speed collision (tests continuous detection determinism)
	_create_high_speed_collision()

func _add_boundaries() -> void:
	var floor_body := StaticBody2D.new()
	var floor_shape := CollisionShape2D.new()
	var floor_rect := RectangleShape2D.new()
	floor_rect.size = Vector2(2000, 40)
	floor_shape.shape = floor_rect
	floor_shape.position = Vector2(0, 20)
	floor_body.add_child(floor_shape)
	floor_body.position = Vector2(576, 628)
	add_child(floor_body)

	# Left wall
	var left_wall := StaticBody2D.new()
	var left_shape := CollisionShape2D.new()
	var left_rect := RectangleShape2D.new()
	left_rect.size = Vector2(40, 1200)
	left_shape.shape = left_rect
	left_shape.position = Vector2(-20, 0)
	left_wall.add_child(left_shape)
	left_wall.position = Vector2(0, 324)
	add_child(left_wall)

	# Right wall
	var right_wall := StaticBody2D.new()
	var right_shape := CollisionShape2D.new()
	var right_rect := RectangleShape2D.new()
	right_rect.size = Vector2(40, 1200)
	right_shape.shape = right_rect
	right_shape.position = Vector2(20, 0)
	right_wall.add_child(right_shape)
	right_wall.position = Vector2(1152, 324)
	add_child(right_wall)

## Scenario 1: Stack of 10 boxes, exercises resting contacts and friction
func _create_box_stack() -> void:
	var box_size := Vector2(30, 30)
	var base_pos := Vector2(200, 608)
	for i in 10:
		var body := RigidBody2D.new()
		body.mass = 1.0
		body.gravity_scale = 1.0
		var shape := CollisionShape2D.new()
		var rect := RectangleShape2D.new()
		rect.size = box_size
		shape.shape = rect
		body.add_child(shape)
		body.position = base_pos + Vector2(0, -(box_size.y + 2) * (i + 1))
		add_child(body)
		bodies.append(body)

## Scenario 2: Bodies with constant forces and torques applied
func _create_force_bodies() -> void:
	for i in 5:
		var body := RigidBody2D.new()
		body.mass = 2.0 + i * 0.5
		body.gravity_scale = 0.5
		var shape := CollisionShape2D.new()
		var rect := RectangleShape2D.new()
		rect.size = Vector2(20, 20)
		shape.shape = rect
		body.add_child(shape)
		body.position = Vector2(400 + i * 60, 300)
		# Apply varied forces that will cause complex interactions
		body.constant_force = Vector2(50.0 * (i - 2), -100.0 + i * 30.0)
		body.constant_torque = 10.0 * (i - 2)
		add_child(body)
		bodies.append(body)

## Scenario 3: Chain of bodies connected by pin joints
func _create_joint_chain() -> void:
	var anchor := StaticBody2D.new()
	var anchor_shape := CollisionShape2D.new()
	var anchor_rect := RectangleShape2D.new()
	anchor_rect.size = Vector2(10, 10)
	anchor_shape.shape = anchor_rect
	anchor.add_child(anchor_shape)
	anchor.position = Vector2(800, 100)
	add_child(anchor)

	var prev_body: PhysicsBody2D = anchor
	for i in 6:
		var body := RigidBody2D.new()
		body.mass = 1.0
		body.gravity_scale = 1.0
		var shape := CollisionShape2D.new()
		var capsule := CapsuleShape2D.new()
		capsule.radius = 5.0
		capsule.height = 30.0
		shape.shape = capsule
		body.add_child(shape)
		body.position = Vector2(800, 100 + (i + 1) * 35)
		add_child(body)
		bodies.append(body)

		# Create pin joint between consecutive bodies
		var joint := PinJoint2D.new()
		joint.position = Vector2(800, 100 + i * 35 + 17)
		joint.node_a = prev_body.get_path()
		joint.node_b = body.get_path()
		joint.softness = 0.0
		add_child(joint)
		joints.append(joint)

		prev_body = body

## Scenario 4: Mixed shapes colliding
func _create_mixed_shapes() -> void:
	# Circle
	var circle_body := RigidBody2D.new()
	circle_body.mass = 3.0
	var circle_shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = 20.0
	circle_shape.shape = circle
	circle_body.add_child(circle_shape)
	circle_body.position = Vector2(550, 100)
	circle_body.linear_velocity = Vector2(50, 0)
	add_child(circle_body)
	bodies.append(circle_body)

	# Convex polygon
	var convex_body := RigidBody2D.new()
	convex_body.mass = 2.5
	var convex_col := CollisionPolygon2D.new()
	convex_col.polygon = PackedVector2Array([
		Vector2(0, -15),
		Vector2(13, -7),
		Vector2(13, 7),
		Vector2(0, 15),
		Vector2(-13, 7),
		Vector2(-13, -7),
	])
	convex_body.add_child(convex_col)
	convex_body.position = Vector2(650, 100)
	convex_body.linear_velocity = Vector2(-30, 10)
	add_child(convex_body)
	bodies.append(convex_body)

	# Large rectangle
	var rect_body := RigidBody2D.new()
	rect_body.mass = 5.0
	var rect_shape := CollisionShape2D.new()
	var rect := RectangleShape2D.new()
	rect.size = Vector2(40, 20)
	rect_shape.shape = rect
	rect_body.add_child(rect_shape)
	rect_body.position = Vector2(600, 200)
	rect_body.angular_velocity = 2.0
	add_child(rect_body)
	bodies.append(rect_body)

## Scenario 5: High-speed collision
func _create_high_speed_collision() -> void:
	var fast_body := RigidBody2D.new()
	fast_body.mass = 1.0
	fast_body.continuous_cd = RigidBody2D.CCD_MODE_CAST_RAY
	var shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = 8.0
	shape.shape = circle
	fast_body.add_child(shape)
	fast_body.position = Vector2(100, 500)
	fast_body.linear_velocity = Vector2(800, -200)
	add_child(fast_body)
	bodies.append(fast_body)

	# Target for the fast body to hit
	var target := RigidBody2D.new()
	target.mass = 10.0
	var target_shape := CollisionShape2D.new()
	var target_rect := RectangleShape2D.new()
	target_rect.size = Vector2(60, 60)
	target_shape.shape = target_rect
	target.add_child(target_shape)
	target.position = Vector2(900, 400)
	add_child(target)
	bodies.append(target)

## Serialize a body's full state using bit-exact float representation.
## Uses PhysicsServer2D.body_get_state() to read directly from the physics
## engine, bypassing any node-side caching or processing.
func _serialize_body(body: RigidBody2D) -> Dictionary:
	var rid := body.get_rid()
	var transform: Transform2D = PhysicsServer2D.body_get_state(rid, PhysicsServer2D.BODY_STATE_TRANSFORM)
	var lin_vel: Vector2 = PhysicsServer2D.body_get_state(rid, PhysicsServer2D.BODY_STATE_LINEAR_VELOCITY)
	var ang_vel: float = PhysicsServer2D.body_get_state(rid, PhysicsServer2D.BODY_STATE_ANGULAR_VELOCITY)
	return {
		"tx_ax": _float_to_hex(transform.x.x),
		"tx_ay": _float_to_hex(transform.x.y),
		"tx_bx": _float_to_hex(transform.y.x),
		"tx_by": _float_to_hex(transform.y.y),
		"tx_ox": _float_to_hex(transform.origin.x),
		"tx_oy": _float_to_hex(transform.origin.y),
		"vx": _float_to_hex(lin_vel.x),
		"vy": _float_to_hex(lin_vel.y),
		"av": _float_to_hex(ang_vel),
	}

## Convert a float to its IEEE 754 bit representation as a hex string.
## This guarantees bit-exact comparison across platforms.
func _float_to_hex(value: float) -> String:
	var buf := PackedByteArray()
	buf.resize(8)
	buf.encode_double(0, value)
	# Encode as big-endian hex for readability and consistent ordering
	var hex := ""
	for i in range(7, -1, -1):
		hex += "%02x" % buf[i]
	return hex

func _finish() -> void:
	var output := {
		"metadata": {
			"simulation_frames": SIMULATION_FRAMES,
			"physics_ticks_per_second": 60,
			"body_count": bodies.size(),
			"os": OS.get_name(),
			"arch": Engine.get_architecture_name(),
		},
		"frames": frame_records,
	}

	var json_string := JSON.stringify(output, "", false)

	# Write to user:// directory
	var file := FileAccess.open(OUTPUT_PATH, FileAccess.WRITE)
	if file:
		file.store_string(json_string)
		file.close()
		print("DETERMINISM_TEST: Output written to %s" % OUTPUT_PATH)
		print("DETERMINISM_TEST: %d frames, %d bodies" % [frame_records.size(), bodies.size()])
		# Also compute a simple hash for quick comparison in logs
		var hash_val := json_string.md5_text()
		print("DETERMINISM_TEST: MD5=%s" % hash_val)
		print("DETERMINISM_TEST: STATUS=SUCCESS")
	else:
		print("DETERMINISM_TEST: ERROR - Failed to open output file")
		print("DETERMINISM_TEST: STATUS=FAILED")

	await get_tree().create_timer(0.5).timeout
	get_tree().quit(0)
