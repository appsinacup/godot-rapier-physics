extends PhysicsUnitTest3D

# Regression test for https://github.com/appsinacup/godot-rapier-physics/issues/600
# PhysicsDirectBodyState3D.inverse_inertia_tensor must scale with the body mass and be
# reported in global orientation, matching Godot's built-in physics server.
#
# The pre-fix code built the tensor from rapier's own inv_principal_inertia, which is
# computed with a mass of 1 per collider, so the tensor never changed when the mass did.
# It also left inverse_inertia at zero and never rotated the tensor into world space.

# Half extents, deliberately anisotropic so the three principal moments differ and a
# rotation is observable in the tensor.
var half_extents := Vector3(1, 2, 3)
var base_mass := 1.0
var mass_factor := 4.0
var body_rotation := PI / 2.0

var body_light: RID
var body_heavy: RID
var body_rotated: RID

func test_description() -> String:
	return """Checks that PhysicsDirectBodyState3D.inverse_inertia_tensor scales with mass,
	matches the analytical box inertia and is expressed in global orientation (issue #600).
	"""

func test_name() -> String:
	return "RigidBody3D | inverse_inertia_tensor vs mass (#600)"

# Analytical inertia of a box of the given half extents, matching what Godot's built-in
# server computes: I = (mass / 3) * (l_j^2 + l_k^2) for each principal axis.
func _expected_inertia(p_mass: float) -> Vector3:
	var x2 := half_extents.x * half_extents.x
	var y2 := half_extents.y * half_extents.y
	var z2 := half_extents.z * half_extents.z
	return (p_mass / 3.0) * Vector3(y2 + z2, x2 + z2, x2 + y2)

func _make_body(p_mass: float, p_transform: Transform3D) -> RID:
	var rid := PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_space(rid, get_world_3d().space)
	PhysicsServer3D.body_set_mode(rid, PhysicsServer3D.BODY_MODE_RIGID)
	# Isolate from the other tests sharing this space.
	PhysicsServer3D.body_set_collision_layer(rid, 0)
	PhysicsServer3D.body_set_collision_mask(rid, 0)

	var shape := PhysicsServer3D.box_shape_create()
	PhysicsServer3D.shape_set_data(shape, half_extents)
	PhysicsServer3D.body_add_shape(rid, shape)

	PhysicsServer3D.body_set_param(rid, PhysicsServer3D.BODY_PARAM_GRAVITY_SCALE, 0.0)
	PhysicsServer3D.body_set_param(rid, PhysicsServer3D.BODY_PARAM_MASS, p_mass)
	PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM, p_transform)
	return rid

func test_start() -> void:
	var checks := func(_p_target, p_monitor: GenericManualMonitor):
		# Create the bodies from inside the monitor callback: at test_start() the runner's
		# SubViewport world is not yet initialized, so get_viewport().world_3d can be null.
		if p_monitor.frame == 1:
			body_light = _make_body(base_mass, Transform3D(Basis(), Vector3(0, 0, 0)))
			body_heavy = _make_body(base_mass * mass_factor, Transform3D(Basis(), Vector3(50, 0, 0)))
			body_rotated = _make_body(base_mass, Transform3D(Basis(Vector3(0, 0, 1), body_rotation), Vector3(100, 0, 0)))

		if p_monitor.frame == 3:
			var ds_light := PhysicsServer3D.body_get_direct_state(body_light)
			var ds_heavy := PhysicsServer3D.body_get_direct_state(body_heavy)
			var ds_rotated := PhysicsServer3D.body_get_direct_state(body_rotated)

			var expected_inv := Vector3.ONE / _expected_inertia(base_mass)
			var tolerance := expected_inv.length() * 0.01

			# inverse_inertia must be the componentwise inverse of the box inertia, not zero
			# (the pre-fix code gated the computation on its own previous value, so it stayed
			# at the Vector3.ZERO it was initialised with).
			p_monitor.add_test("inverse_inertia matches the analytical box inertia")
			var inv_inertia_ok: bool = Utils.vec3_equals(ds_light.inverse_inertia, expected_inv, tolerance)
			if not inv_inertia_ok:
				p_monitor.add_test_error("Expected %v, got %v" % [expected_inv, ds_light.inverse_inertia])
			p_monitor.add_test_result(inv_inertia_ok)

			# The tensor of an axis aligned box is the diagonal of the inverse inertia.
			p_monitor.add_test("inverse_inertia_tensor matches the analytical box inertia")
			var diag_light := _basis_diagonal(ds_light.inverse_inertia_tensor)
			var diag_ok: bool = Utils.vec3_equals(diag_light, expected_inv, tolerance)
			if not diag_ok:
				p_monitor.add_test_error("Expected diagonal %v, got tensor %s" % [expected_inv, ds_light.inverse_inertia_tensor])
			p_monitor.add_test_result(diag_ok)

			# The actual regression: inertia is proportional to mass, so the inverse tensor of
			# a body that is `mass_factor` times heavier must be `mass_factor` times smaller.
			p_monitor.add_test("inverse_inertia_tensor scales with mass")
			var diag_heavy := _basis_diagonal(ds_heavy.inverse_inertia_tensor)
			var expected_heavy := diag_light / mass_factor
			var scale_ok: bool = Utils.vec3_equals(diag_heavy, expected_heavy, tolerance)
			if not scale_ok:
				p_monitor.add_test_error("Expected diagonal %v for mass %f, got %v" % [expected_heavy, base_mass * mass_factor, diag_heavy])
			p_monitor.add_test_result(scale_ok)

			# Godot reports the tensor in global orientation, so rotating the body by 90
			# degrees around Z must swap the X and Y entries of the diagonal.
			p_monitor.add_test("inverse_inertia_tensor is expressed in global orientation")
			var diag_rotated := _basis_diagonal(ds_rotated.inverse_inertia_tensor)
			var expected_rotated := Vector3(diag_light.y, diag_light.x, diag_light.z)
			var rotation_ok: bool = Utils.vec3_equals(diag_rotated, expected_rotated, tolerance)
			if not rotation_ok:
				p_monitor.add_test_error("Expected diagonal %v when rotated, got %v" % [expected_rotated, diag_rotated])
			p_monitor.add_test_result(rotation_ok)

			PhysicsServer3D.free_rid(body_light)
			PhysicsServer3D.free_rid(body_heavy)
			PhysicsServer3D.free_rid(body_rotated)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks)

func _basis_diagonal(p_basis: Basis) -> Vector3:
	return Vector3(p_basis[0][0], p_basis[1][1], p_basis[2][2])
