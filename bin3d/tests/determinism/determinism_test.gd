extends Node3D
## Cross-platform determinism test (3D).
##
## Runs a reproducible 3D physics simulation for a fixed number of frames,
## records the exact state of every body each frame using bit-exact float
## representation, and writes the result to a JSON file.
##
## The CI pipeline runs this on multiple OS/architecture combinations and
## compares the output files byte-for-byte.

const SIMULATION_FRAMES := 300
const OUTPUT_PATH := "user://determinism_output.json"

var bodies: Array[RigidBody3D] = []
var joints: Array[Joint3D] = []
var frame_records: Array[Dictionary] = []
var current_frame := 0
var simulation_started := false

func _ready() -> void:
	# Verify the rapier extension is loaded — abort if not
	var engine_name = ProjectSettings.get("physics/3d/physics_engine")
	if engine_name != "Rapier3D":
		print("DETERMINISM_TEST: ERROR - Rapier3D not loaded (engine=%s)" % engine_name)
		print("DETERMINISM_TEST: STATUS=FAILED")
		await get_tree().create_timer(0.5).timeout
		get_tree().quit(1)
		return

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
		simulation_started = false
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

	# Scenario 4: Different shapes interacting (sphere vs convex vs box)
	_create_mixed_shapes()

	# Scenario 5: High-speed collision
	_create_high_speed_collision()

func _add_boundaries() -> void:
	# Floor
	var floor_body := StaticBody3D.new()
	var floor_shape := CollisionShape3D.new()
	var floor_box := BoxShape3D.new()
	floor_box.size = Vector3(50, 1, 50)
	floor_shape.shape = floor_box
	floor_body.add_child(floor_shape)
	floor_body.position = Vector3(0, -0.5, 0)
	add_child(floor_body)

	# Walls
	for wall_data in [
		Vector3(-25, 5, 0), Vector3(1, 10, 50),   # left
		Vector3(25, 5, 0), Vector3(1, 10, 50),    # right
		Vector3(0, 5, -25), Vector3(50, 10, 1),   # back
		Vector3(0, 5, 25), Vector3(50, 10, 1),    # front
	]:
		pass
	_add_wall(Vector3(-25, 5, 0), Vector3(1, 10, 50))
	_add_wall(Vector3(25, 5, 0), Vector3(1, 10, 50))
	_add_wall(Vector3(0, 5, -25), Vector3(50, 10, 1))
	_add_wall(Vector3(0, 5, 25), Vector3(50, 10, 1))

func _add_wall(pos: Vector3, size: Vector3) -> void:
	var wall := StaticBody3D.new()
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	wall.add_child(shape)
	wall.position = pos
	add_child(wall)

## Scenario 1: Stack of 8 boxes
func _create_box_stack() -> void:
	var box_size := Vector3(1, 1, 1)
	var base_pos := Vector3(-5, 0.5, 0)
	for i in 8:
		var body := RigidBody3D.new()
		body.mass = 1.0
		body.gravity_scale = 1.0
		var shape := CollisionShape3D.new()
		var box := BoxShape3D.new()
		box.size = box_size
		shape.shape = box
		body.add_child(shape)
		body.position = base_pos + Vector3(0, (box_size.y + 0.05) * i, 0)
		add_child(body)
		bodies.append(body)

## Scenario 2: Bodies with constant forces and torques applied
func _create_force_bodies() -> void:
	for i in 5:
		var body := RigidBody3D.new()
		body.mass = 2.0 + i * 0.5
		body.gravity_scale = 0.5
		var shape := CollisionShape3D.new()
		var box := BoxShape3D.new()
		box.size = Vector3(0.8, 0.8, 0.8)
		shape.shape = box
		body.add_child(shape)
		body.position = Vector3(0 + i * 2, 3, -5)
		body.constant_force = Vector3(2.0 * (i - 2), -5.0 + i * 2.0, 1.0 * i)
		body.constant_torque = Vector3(0.5 * (i - 2), 0.3 * i, -0.2 * (i - 1))
		add_child(body)
		bodies.append(body)

## Scenario 3: Chain of bodies connected by pin joints
func _create_joint_chain() -> void:
	var anchor := StaticBody3D.new()
	var anchor_shape := CollisionShape3D.new()
	var anchor_box := BoxShape3D.new()
	anchor_box.size = Vector3(0.3, 0.3, 0.3)
	anchor_shape.shape = anchor_box
	anchor.add_child(anchor_shape)
	anchor.position = Vector3(5, 8, 0)
	add_child(anchor)

	var prev_body: PhysicsBody3D = anchor
	for i in 6:
		var body := RigidBody3D.new()
		body.mass = 1.0
		body.gravity_scale = 1.0
		var shape := CollisionShape3D.new()
		var capsule := CapsuleShape3D.new()
		capsule.radius = 0.15
		capsule.height = 1.0
		shape.shape = capsule
		body.add_child(shape)
		body.position = Vector3(5, 8 - (i + 1) * 1.2, 0)
		add_child(body)
		bodies.append(body)

		# Create pin joint between consecutive bodies
		var joint := PinJoint3D.new()
		joint.position = Vector3(5, 8 - i * 1.2 - 0.6, 0)
		joint.node_a = prev_body.get_path()
		joint.node_b = body.get_path()
		add_child(joint)
		joints.append(joint)

		prev_body = body

## Scenario 4: Mixed shapes colliding
func _create_mixed_shapes() -> void:
	# Sphere
	var sphere_body := RigidBody3D.new()
	sphere_body.mass = 3.0
	var sphere_shape := CollisionShape3D.new()
	var sphere := SphereShape3D.new()
	sphere.radius = 0.7
	sphere_shape.shape = sphere
	sphere_body.add_child(sphere_shape)
	sphere_body.position = Vector3(-3, 2, -3)
	sphere_body.linear_velocity = Vector3(3, 0, 2)
	add_child(sphere_body)
	bodies.append(sphere_body)

	# Convex hull (approximated with a cylinder)
	var cyl_body := RigidBody3D.new()
	cyl_body.mass = 2.5
	var cyl_shape := CollisionShape3D.new()
	var cyl := CylinderShape3D.new()
	cyl.radius = 0.5
	cyl.height = 1.5
	cyl_shape.shape = cyl
	cyl_body.add_child(cyl_shape)
	cyl_body.position = Vector3(0, 2, -3)
	cyl_body.linear_velocity = Vector3(-2, 0, 1)
	add_child(cyl_body)
	bodies.append(cyl_body)

	# Large box
	var box_body := RigidBody3D.new()
	box_body.mass = 5.0
	var box_shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(1.5, 0.8, 1.2)
	box_shape.shape = box
	box_body.add_child(box_shape)
	box_body.position = Vector3(-1, 3, -3)
	box_body.angular_velocity = Vector3(1.0, 0.5, -0.3)
	add_child(box_body)
	bodies.append(box_body)

## Scenario 5: High-speed collision
func _create_high_speed_collision() -> void:
	var fast_body := RigidBody3D.new()
	fast_body.mass = 1.0
	fast_body.continuous_cd = true
	var shape := CollisionShape3D.new()
	var sphere := SphereShape3D.new()
	sphere.radius = 0.3
	shape.shape = sphere
	fast_body.add_child(shape)
	fast_body.position = Vector3(-10, 2, 5)
	fast_body.linear_velocity = Vector3(30, -5, -10)
	add_child(fast_body)
	bodies.append(fast_body)

	# Target
	var target := RigidBody3D.new()
	target.mass = 10.0
	var target_shape := CollisionShape3D.new()
	var target_box := BoxShape3D.new()
	target_box.size = Vector3(2, 2, 2)
	target_shape.shape = target_box
	target.add_child(target_shape)
	target.position = Vector3(5, 2, 0)
	add_child(target)
	bodies.append(target)

## Serialize a body's full state using bit-exact float representation.
## Uses PhysicsServer3D.body_get_state() to read directly from the physics
## engine, bypassing any node-side caching or processing.
func _serialize_body(body: RigidBody3D) -> Dictionary:
	var rid := body.get_rid()
	var transform: Transform3D = PhysicsServer3D.body_get_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM)
	var lin_vel: Vector3 = PhysicsServer3D.body_get_state(rid, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY)
	var ang_vel: Vector3 = PhysicsServer3D.body_get_state(rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY)
	var basis := transform.basis
	return {
		"bxx": _float_to_hex(basis.x.x),
		"bxy": _float_to_hex(basis.x.y),
		"bxz": _float_to_hex(basis.x.z),
		"byx": _float_to_hex(basis.y.x),
		"byy": _float_to_hex(basis.y.y),
		"byz": _float_to_hex(basis.y.z),
		"bzx": _float_to_hex(basis.z.x),
		"bzy": _float_to_hex(basis.z.y),
		"bzz": _float_to_hex(basis.z.z),
		"ox": _float_to_hex(transform.origin.x),
		"oy": _float_to_hex(transform.origin.y),
		"oz": _float_to_hex(transform.origin.z),
		"vx": _float_to_hex(lin_vel.x),
		"vy": _float_to_hex(lin_vel.y),
		"vz": _float_to_hex(lin_vel.z),
		"avx": _float_to_hex(ang_vel.x),
		"avy": _float_to_hex(ang_vel.y),
		"avz": _float_to_hex(ang_vel.z),
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
