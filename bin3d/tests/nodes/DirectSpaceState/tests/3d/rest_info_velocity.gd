extends PhysicsUnitTest3D

# Regression test for the fix to `_rest_info` point velocity.
# `PhysicsDirectSpaceState3D.get_rest_info` reports `linear_velocity`: the velocity of the
# collided body AT the contact point. For a rigid body this is:
#     linear_velocity = body_linear_velocity + angular_velocity x (point - global_com)
# where the global center of mass is the LOCAL center of mass transformed by the full body
# transform (rotation included), not merely offset by the origin. The pre-fix code used
# `origin + local_com`, which is wrong as soon as the body is rotated.
#
# This spins a body that has a custom center of mass and a rotation, queries the rest info
# at a contact point, and checks the reported velocity against the correct formula. It also
# asserts the reported velocity does NOT match the pre-fix (origin + local_com) computation.

var simulation_duration := 10.0

var body_position := Vector3(0, 0, 0)
var body_rotation := PI / 2.0
var local_com := Vector3(10, 0, 0)
var angular_velocity := Vector3(0, 0, 2.0)

var body_rid: RID
var query_shape: RID
# A high, unique collision layer so the query only sees this body and no other test pushes it.
var query_layer := 1 << 19

func test_description() -> String:
	return """Checks that get_rest_info reports the contact-point velocity using the global
	center of mass (transform * local_com), for a rotating body with a custom center of mass.
	"""

func test_name() -> String:
	return "DirectSpaceState3D | get_rest_info point velocity about center of mass"

func test_start() -> void:
	var checks := func(_p_target, p_monitor: GenericManualMonitor):
		# Create the body from inside the monitor callback: at test_start() the runner's
		# SubViewport world is not yet initialized, so get_viewport().world_3d can be null.
		if p_monitor.frame == 1:
			body_rid = PhysicsServer3D.body_create()
			PhysicsServer3D.body_set_space(body_rid, get_world_3d().space)
			PhysicsServer3D.body_set_mode(body_rid, PhysicsServer3D.BODY_MODE_RIGID)
			# Isolate from the other tests sharing this space: a unique collision layer so only
			# our query detects it, and mask 0 so nothing pushes it while it spins.
			PhysicsServer3D.body_set_collision_layer(body_rid, query_layer)
			PhysicsServer3D.body_set_collision_mask(body_rid, 0)

			var body_shape := PhysicsServer3D.box_shape_create()
			PhysicsServer3D.shape_set_data(body_shape, Vector3(40, 40, 40))
			PhysicsServer3D.body_add_shape(body_rid, body_shape)

			PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BODY_PARAM_GRAVITY_SCALE, 0.0)
			PhysicsServer3D.body_set_param(body_rid, PhysicsServer3D.BODY_PARAM_CENTER_OF_MASS, local_com)

			var xform := Transform3D(Basis(Vector3(0, 0, 1), body_rotation), body_position)
			PhysicsServer3D.body_set_state(body_rid, PhysicsServer3D.BODY_STATE_TRANSFORM, xform)
			# Pure spin about the center of mass, no linear velocity.
			PhysicsServer3D.body_set_state(body_rid, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, Vector3.ZERO)
			PhysicsServer3D.body_set_state(body_rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, angular_velocity)

			query_shape = PhysicsServer3D.box_shape_create()
			PhysicsServer3D.shape_set_data(query_shape, Vector3(30, 30, 30))
			return

		# Query after a couple of physics steps so velocities are settled.
		if p_monitor.frame != 3:
			return

		var ds := PhysicsServer3D.body_get_direct_state(body_rid)
		var d_space := get_world_3d().direct_space_state

		var query := PhysicsShapeQueryParameters3D.new()
		query.shape_rid = query_shape
		query.collide_with_bodies = true
		query.collision_mask = query_layer
		query.transform = Transform3D(Basis.IDENTITY, body_position)
		var info := d_space.get_rest_info(query)

		p_monitor.add_test("get_rest_info reports a collision with the body")
		var hit: bool = not info.is_empty() and info.get("rid", RID()) == body_rid
		p_monitor.add_test_result(hit)

		if hit:
			var point: Vector3 = info["point"]
			var reported: Vector3 = info["linear_velocity"]

			# Correct: the point velocity uses the global center of mass (full transform).
			var global_com: Vector3 = ds.transform * local_com
			var rel: Vector3 = point - global_com
			var expected: Vector3 = ds.linear_velocity + angular_velocity.cross(rel)

			# Tolerance absorbs the sub-step skew between the query's internal snapshot and
			# the transform read here (about one physics step of rotation); it stays far below
			# the pre-fix error, which is an order of magnitude larger (see the check below).
			p_monitor.add_test("Reported point velocity matches the global center of mass formula")
			var ok: bool = Utils.vec3_equals(reported, expected, 3.0)
			if not ok:
				p_monitor.add_test_error("Expected %v, got %v (point %v, global_com %v)" % [expected, reported, point, global_com])
			p_monitor.add_test_result(ok)

			# The pre-fix code used (origin + local_com) as the center of mass. With a
			# rotation that differs from the correct global_com, so the reported velocity
			# must NOT match that computation.
			var wrong_com: Vector3 = ds.transform.origin + local_com
			var wrong_rel: Vector3 = point - wrong_com
			var wrong_expected: Vector3 = ds.linear_velocity + angular_velocity.cross(wrong_rel)
			p_monitor.add_test("Reported point velocity is not the pre-fix (origin + local_com) value")
			p_monitor.add_test_result(not Utils.vec3_equals(reported, wrong_expected, 3.0))

		PhysicsServer3D.free_rid(query_shape)
		PhysicsServer3D.free_rid(body_rid)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
