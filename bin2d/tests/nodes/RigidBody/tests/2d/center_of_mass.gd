extends PhysicsUnitTest2D

# Regression test for https://github.com/appsinacup/godot-rapier-physics/issues/550
# PhysicsDirectBodyState2D.center_of_mass must return the center of mass offset relative
# to the body origin, expressed in global orientation (rotated by the body basis but NOT
# translated by the body origin) - matching Godot's built-in physics server.

var local_com := Vector2(3, 0)
# A transform with both a rotation and a translation, so the expected global offset differs
# from both the local offset and the absolute global position of the center of mass.
var body_transform := Transform2D(PI / 2.0, Vector2(100, 50))

func test_description() -> String:
	return """Checks that PhysicsDirectBodyState2D.center_of_mass returns the center of mass
	offset relative to the body origin, in global orientation (issue #550).
	"""

func test_name() -> String:
	return "RigidBody2D | center_of_mass semantics (#550)"

func test_start() -> void:
	var body_rid := PhysicsServer2D.body_create()
	PhysicsServer2D.body_set_space(body_rid, get_viewport().world_2d.space)
	PhysicsServer2D.body_set_mode(body_rid, PhysicsServer2D.BODY_MODE_RIGID)
	PhysicsServer2D.body_set_param(body_rid, PhysicsServer2D.BODY_PARAM_GRAVITY_SCALE, 0.0)
	PhysicsServer2D.body_set_param(body_rid, PhysicsServer2D.BODY_PARAM_CENTER_OF_MASS, local_com)
	PhysicsServer2D.body_set_state(body_rid, PhysicsServer2D.BODY_STATE_TRANSFORM, body_transform)

	var checks := func(_p_target, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 2:
			var ds := PhysicsServer2D.body_get_direct_state(body_rid)

			p_monitor.add_test("Direct body state is available")
			p_monitor.add_test_result(ds != null)

			if ds != null:
				p_monitor.add_test("center_of_mass_local is the local offset")
				p_monitor.add_test_result(ds.center_of_mass_local.is_equal_approx(local_com))

				# center_of_mass must be the local offset rotated into global orientation,
				# i.e. transform.basis_xform(local_com) - rotated but NOT translated.
				var expected: Vector2 = body_transform.basis_xform(local_com)
				p_monitor.add_test("center_of_mass is the offset rotated into global orientation")
				if not ds.center_of_mass.is_equal_approx(expected):
					p_monitor.add_test_error("Expected %s, got %s" % [expected, ds.center_of_mass])
				p_monitor.add_test_result(ds.center_of_mass.is_equal_approx(expected))

				# It must NOT be the absolute global position of the center of mass
				# (which is what the pre-fix code incorrectly returned).
				p_monitor.add_test("center_of_mass is not the absolute global position")
				p_monitor.add_test_result(not ds.center_of_mass.is_equal_approx(body_transform * local_com))

			PhysicsServer2D.free_rid(body_rid)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks)
