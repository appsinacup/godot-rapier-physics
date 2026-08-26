extends PhysicsUnitTest3D

var simulation_duration := 10

# A property applied to a body that is already simulating has to end up in the same state as the
# same property applied before the body ever moved. Anything that only reaches rapier at creation
# time shows up here as the two bodies drifting apart.

# A property change can leave the solver re-converging for a while, so the bodies are given time
# to come back to rest before the stimulus -- otherwise this measures the transient, not the change.
const SETTLE_FRAME := 20
const STIMULUS_FRAME := 100
const CHECK_FRAME := 170
const POSITION_TOLERANCE := 0.02
const ROTATION_TOLERANCE := 0.02

var checks := []
var lane := 0.0

func test_description() -> String:
	return """Checks that a body property changed while the body is already in the space reaches
	rapier, by comparing against an identical body given the same property at creation.
	"""

func test_name() -> String:
	return "RigidBody3D | properties changed at runtime reach the simulation"

func test_start() -> void:
	var push := func(b: RigidBody3D): b.apply_central_impulse(Vector3(3, 0, 0))
	var spin := func(b: RigidBody3D): b.apply_torque_impulse(Vector3(0, 0, 2))
	var offset_push := func(b: RigidBody3D): b.apply_impulse(Vector3(3, 0, 0), Vector3(0, 0.5, 0))

	add_check("mass", func(b): b.mass = 5.0, push)
	add_check("inertia", func(b): b.inertia = Vector3(8, 8, 8), spin)
	# Free space: the response to an off-center impulse depends directly on the center of mass,
	# and without contacts the comparison is deterministic on every engine.
	add_check("center of mass", func(b):
		b.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
		b.center_of_mass = Vector3(0, 0.45, 0), offset_push, true, false)
	add_check("gravity scale", func(b): b.gravity_scale = 0.0, push)
	add_check("friction", func(b): b.physics_material_override.friction = 0.0, push)
	add_check("bounce", func(b): b.physics_material_override.bounce = 0.9, push)
	add_check("linear damp", func(b): b.linear_damp = 4.0, push)
	add_check("angular damp", func(b): b.angular_damp = 4.0, spin)
	add_check("scale", func(b): b.scale = Vector3(2, 2, 2), push)
	add_check("freeze", func(b):
		b.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
		b.freeze = true, push)
	add_check("axis lock", func(b): b.axis_lock_linear_x = true, push)
	add_check("custom integrator", func(b): b.custom_integrator = true, push)
	add_check("shape size", func(b): (b.get_child(0).shape as BoxShape3D).size = Vector3(2, 2, 2), push)
	add_check("collision mask", func(b):
		b.collision_layer = 4
		b.collision_mask = 4, push, true)
	add_check("shape disabled", func(b): (b.get_child(0) as CollisionShape3D).disabled = true, push, true)

	var run = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == SETTLE_FRAME:
			for c in checks:
				for key in ["early", "late"]:
					var b: RigidBody3D = c[key]
					b.linear_velocity = Vector3.ZERO
					b.angular_velocity = Vector3.ZERO
				c["apply"].call(c["late"])

		if p_monitor.frame == STIMULUS_FRAME:
			for c in checks:
				c["stimulus"].call(c["early"])
				c["stimulus"].call(c["late"])

		if p_monitor.frame == CHECK_FRAME:
			for c in checks:
				var early: RigidBody3D = c["early"]
				var late: RigidBody3D = c["late"]
				var moved_early: Vector3 = early.global_position - c["early_start"]
				var moved_late: Vector3 = late.global_position - c["late_start"]
				var position_drift: float = (moved_early - moved_late).length()
				var rotation_drift := (early.global_basis.get_euler() - late.global_basis.get_euler()).length()
				p_monitor.add_test("%s applied at runtime matches applied at creation" % c["name"])
				if c["name"] == "center of mass":
					# Godot Physics itself responds differently to a center of mass applied at
					# runtime than to one applied at creation, even free of contacts. Rapier
					# propagates it fully, so the parity holds only there.
					p_monitor.add_test_engine_expected_to_fail(
						["GodotPhysics3D", "Godot Physics 3D"]
					)
				if position_drift >= POSITION_TOLERANCE:
					p_monitor.add_test_error("position drifted by %f" % position_drift)
				if rotation_drift >= ROTATION_TOLERANCE:
					p_monitor.add_test_error("rotation drifted by %f" % rotation_drift)
				p_monitor.add_test_result(position_drift < POSITION_TOLERANCE and rotation_drift < ROTATION_TOLERANCE)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)

# `floating` swaps the floor for a pair of walls, so that a property which stops the body colliding
# lets it pass through a wall instead of dropping it -- otherwise the two bodies would start
# falling at different times and drift apart for reasons that have nothing to do with the property.
func add_check(
	p_name: String, p_apply: Callable, p_stimulus: Callable, p_floating := false, p_walls := true
) -> void:
	lane += 12.0
	if p_floating:
		if p_walls:
			add_wall(Vector3(-2, 0, lane))
			add_wall(Vector3(10, 0, lane))
	else:
		add_floor(lane)
	var early := add_body(Vector3(-6, 0, lane), p_floating)
	var late := add_body(Vector3(6, 0, lane), p_floating)
	p_apply.call(early)
	checks.append({
		"name": p_name, "apply": p_apply, "stimulus": p_stimulus,
		"early": early, "late": late,
		"early_start": early.global_position, "late_start": late.global_position,
	})

func add_floor(p_lane: float) -> void:
	var floor_body := StaticBody3D.new()
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new(); box.size = Vector3(60, 1, 6)
	shape.shape = box
	floor_body.add_child(shape)
	floor_body.position = Vector3(0, -1, p_lane)
	var material := PhysicsMaterial.new(); material.friction = 1.0; material.bounce = 0.0
	floor_body.physics_material_override = material
	add_child(floor_body)

func add_wall(p_position: Vector3) -> void:
	var wall := StaticBody3D.new()
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new(); box.size = Vector3(1, 6, 6)
	shape.shape = box
	wall.add_child(shape)
	wall.position = p_position
	add_child(wall)

func add_body(p_position: Vector3, p_floating: bool) -> RigidBody3D:
	var body := RigidBody3D.new()
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new(); box.size = Vector3(1, 1, 1)
	shape.shape = box
	body.add_child(shape)
	body.position = p_position
	body.can_sleep = false
	if p_floating:
		body.gravity_scale = 0.0
	var material := PhysicsMaterial.new(); material.friction = 0.5; material.bounce = 0.0
	body.physics_material_override = material
	add_child(body)
	return body
