extends PhysicsUnitTest3D

# Checks that a body made of several differently sized shapes gets the same mass properties
# as Godot's built-in physics server, which distributes the body mass over the shapes in
# proportion to their volume:
#     mass_i = volume_i * mass / total_volume
# and then sums the shape inertias about the common center of mass (parallel axis theorem).
#
# The shape mass properties used to be obtained by forcing every collider to a mass of 1,
# which spread the mass equally over the shapes instead of by volume. That gave the right
# answer only when all shapes had the same volume.

# Two boxes of very different volume (8 vs 64), offset along X only, so the combined inertia
# tensor stays diagonal and the expected values are fully analytical.
var half_extents_a := Vector3(1, 1, 1)
var half_extents_b := Vector3(2, 2, 2)
var origin_a := Vector3(-3, 0, 0)
var origin_b := Vector3(1, 0, 0)
var mass := 10.0

var body_rid: RID

func test_description() -> String:
	return """Checks that the mass of a multi shape body is distributed over the shapes in
	proportion to their volume, matching Godot's built-in physics server.
	"""

func test_name() -> String:
	return "RigidBody3D | multi shape inertia distribution"

func _box_volume(p_half_extents: Vector3) -> float:
	return 8.0 * p_half_extents.x * p_half_extents.y * p_half_extents.z

# Godot's GodotBoxShape3D::get_moment_of_inertia.
func _box_moment_of_inertia(p_half_extents: Vector3, p_mass: float) -> Vector3:
	var x2 := p_half_extents.x * p_half_extents.x
	var y2 := p_half_extents.y * p_half_extents.y
	var z2 := p_half_extents.z * p_half_extents.z
	return (p_mass / 3.0) * Vector3(y2 + z2, x2 + z2, x2 + y2)

func test_start() -> void:
	var volume_a := _box_volume(half_extents_a)
	var volume_b := _box_volume(half_extents_b)
	var total_volume := volume_a + volume_b

	var mass_a := volume_a * mass / total_volume
	var mass_b := volume_b * mass / total_volume

	# Godot assumes the shape origin is also the shape's center of mass.
	var expected_com := (mass_a * origin_a + mass_b * origin_b) / mass

	# Sum the shape inertias about the common center of mass. Both shifts are along X, so the
	# off diagonal terms of the outer product vanish and the tensor stays diagonal.
	var expected_inertia := Vector3.ZERO
	for shape in [[half_extents_a, origin_a, mass_a], [half_extents_b, origin_b, mass_b]]:
		var shift: Vector3 = shape[1] - expected_com
		var shift_term := Vector3.ONE * shift.dot(shift) - shift * shift
		expected_inertia += _box_moment_of_inertia(shape[0], shape[2]) + shift_term * shape[2]

	var checks := func(_p_target, p_monitor: GenericManualMonitor):
		# Create the body from inside the monitor callback: at test_start() the runner's
		# SubViewport world is not yet initialized, so get_viewport().world_3d can be null.
		if p_monitor.frame == 1:
			body_rid = PhysicsServer3D.body_create()
			PhysicsServer3D.body_set_space(body_rid, get_world_3d().space)
			PhysicsServer3D.body_set_mode(body_rid, PhysicsServer3D.BODY_MODE_RIGID)
			PhysicsServer3D.body_set_collision_layer(body_rid, 0)
			PhysicsServer3D.body_set_collision_mask(body_rid, 0)

			var shape_a := PhysicsServer3D.box_shape_create()
			PhysicsServer3D.shape_set_data(shape_a, half_extents_a)
			PhysicsServer3D.body_add_shape(body_rid, shape_a, Transform3D(Basis(), origin_a))

			var shape_b := PhysicsServer3D.box_shape_create()
			PhysicsServer3D.shape_set_data(shape_b, half_extents_b)
			PhysicsServer3D.body_add_shape(body_rid, shape_b, Transform3D(Basis(), origin_b))

			PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BODY_PARAM_GRAVITY_SCALE, 0.0)
			PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BODY_PARAM_MASS, mass)

		if p_monitor.frame == 3:
			var ds := PhysicsServer3D.body_get_direct_state(body_rid)

			# The center of mass is the volume weighted average of the shape origins.
			p_monitor.add_test("Center of mass is weighted by shape volume")
			var com_ok: bool = Utils.vec3_equals(ds.center_of_mass_local, expected_com, 0.001)
			if not com_ok:
				p_monitor.add_test_error("Expected %v, got %v" % [expected_com, ds.center_of_mass_local])
			p_monitor.add_test_result(com_ok)

			var reported_inertia: Vector3 = PhysicsServer3D.body_get_param(body_rid, PhysicsServer3D.BODY_PARAM_INERTIA)
			p_monitor.add_test("Inertia matches Godot's volume weighted distribution")
			var inertia_ok: bool = Utils.vec3_equals(reported_inertia, expected_inertia, expected_inertia.length() * 0.01)
			if not inertia_ok:
				p_monitor.add_test_error("Expected %v, got %v" % [expected_inertia, reported_inertia])
			p_monitor.add_test_result(inertia_ok)

			# The inverse inertia tensor has to stay consistent with that inertia.
			p_monitor.add_test("Inverse inertia tensor is consistent with the inertia")
			var tensor: Basis = ds.inverse_inertia_tensor
			var diagonal := Vector3(tensor[0][0], tensor[1][1], tensor[2][2])
			var expected_inverse := Vector3.ONE / expected_inertia
			var tensor_ok: bool = Utils.vec3_equals(diagonal, expected_inverse, expected_inverse.length() * 0.01)
			if not tensor_ok:
				p_monitor.add_test_error("Expected diagonal %v, got %v" % [expected_inverse, diagonal])
			p_monitor.add_test_result(tensor_ok)

			PhysicsServer3D.free_rid(body_rid)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks)
